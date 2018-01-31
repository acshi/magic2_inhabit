#include "vfh_star.h"
#include "gui.h"
#include "common/general_search.h"

typedef struct expansion {
    vfh_plus_t *vfh;
    int i;
} expansion_t;

static void circle_add_lines(zarray_t *lines, int cx, int cy, int x, int y)
{
    int line_top[3] = {cx - x, cx + x, cy + y};
    zarray_add(lines, line_top);

    if (y != 0) {
        int line_bottom[3] = {cx - x, cx + x, cy + y};
        zarray_add(lines, line_bottom);
    }
}

// based on https://stackoverflow.com/questions/10878209/midpoint-circle-algorithm-for-filled-circles
zarray_t *rasterize_circle_lines(int cx, int cy, int r)
{
    zarray_t *lines = zarray_create(sizeof(int[3]));

    int error = -r;
    int x = r;
    int y = 0;

    while (x >= y) {
        int last_y = y;
        error += 2 * y + 1;
        y++;
        circle_add_lines(lines, cx, cy, x, last_y);
        if (error >= 0) {
            if (x != last_y) {
                circle_add_lines(lines, cx, cy, last_y, x);
            }
            error -= 2 * x + 1;
            x--;
        }
    }
    return lines;
}

void initialize_vfh_star(drive_to_wp_state_t *state, config_t *config)
{
    state->polar_sections = config_require_int(config, "vfh_star.polar_sections");
    require_value_nonnegative(state->polar_sections, "vfh_star.polar_sections");

    state->wide_opening_size = config_require_int(config, "vfh_star.wide_opening_size");
    require_value_nonnegative(state->wide_opening_size, "vfh_star.wide_opening_size");

    state->active_diameter = config_require_double(config, "vfh_star.active_diameter");
    require_value_nonnegative(state->active_diameter, "vfh_star.active_diameter");

    state->max_magnitude = config_require_double(config, "vfh_star.max_magnitude");
    require_value_nonnegative(state->max_magnitude, "vfh_star.max_magnitude");

    state->planning_clearance = config_require_double(config, "vfh_star.planning_clearance");
    require_value_nonnegative(state->planning_clearance, "vfh_star.planning_clearance");

    state->polar_density_traversable = config_require_double(config, "vfh_star.polar_density_traversable");
    require_value_nonnegative(state->polar_density_traversable, "vfh_star.polar_density_traversable");

    state->polar_density_occupied = config_require_double(config, "vfh_star.polar_density_occupied");
    require_value_nonnegative(state->polar_density_occupied, "vfh_star.polar_density_occupied");

    state->step_distance = config_require_double(config, "vfh_star.step_distance");
    require_value_nonnegative(state->step_distance, "vfh_star.step_distance");

    state->goal_depth = config_require_int(config, "vfh_star.goal_depth");
    require_value_nonnegative(state->goal_depth, "vfh_star.goal_depth");

    state->discount_factor = config_require_double(config, "vfh_star.discount_factor");
    require_value_nonnegative(state->discount_factor, "vfh_star.discount_factor");

    state->cost_goal_oriented = config_require_double(config, "vfh_star.cost_goal_oriented");
    require_value_nonnegative(state->cost_goal_oriented, "vfh_star.cost_goal_oriented");

    state->cost_smooth_path = config_require_double(config, "vfh_star.cost_smooth_path");
    require_value_nonnegative(state->cost_smooth_path, "vfh_star.cost_smooth_path");

    state->cost_smooth_commands = config_require_double(config, "vfh_star.cost_smooth_commands");
    require_value_nonnegative(state->cost_smooth_commands, "vfh_star.cost_smooth_commands");

    state->cost_proj_goal_oriented = config_require_double(config, "vfh_star.cost_proj_goal_oriented");
    require_value_nonnegative(state->cost_proj_goal_oriented, "vfh_star.cost_proj_goal_oriented");

    state->cost_proj_smooth_path = config_require_double(config, "vfh_star.cost_proj_smooth_path");
    require_value_nonnegative(state->cost_proj_smooth_path, "vfh_star.cost_proj_smooth_path");

    state->cost_proj_smooth_commands = config_require_double(config, "vfh_star.cost_proj_smooth_commands");
    require_value_nonnegative(state->cost_proj_smooth_commands, "vfh_star.cost_proj_smooth_commands");
}

void delayed_initialize_vfh_star(drive_to_wp_state_t *state)
{
    grid_map_t *gm = state->last_grid_map;

    int r = (int)(state->active_diameter / gm->meters_per_pixel + 0.5);

    double robot_rs = state->vehicle_width / 2 + state->planning_clearance;
    double scale_b = (state->max_magnitude - 1) / sq((state->active_diameter - 1) / 2);

    zarray_t *circle_lines = rasterize_circle_lines(0, 0, r);
    state->precomp_circle_lines = circle_lines;

    int pixels_in_circle = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        pixels_in_circle += line[1] - line[0] + 1;
    }

    state->precomp_radians = calloc(pixels_in_circle, sizeof(double));
    state->precomp_enlargements = calloc(pixels_in_circle, sizeof(double));
    state->precomp_magnitudes = calloc(pixels_in_circle, sizeof(double));

    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++) {
            double rads = M_PI + atan2(y, x);
            double dist_sq = (x * x + y * y);
            double dist = sqrt(dist_sq);
            double magnitude = (state->max_magnitude - scale_b * dist_sq);
            double enlargement_rad = asin(robot_rs / dist);
            state->precomp_radians[pixel_on] = rads;
            state->precomp_enlargements[pixel_on] = enlargement_rad;
            state->precomp_magnitudes[pixel_on] = magnitude;
        }
    }
}

void update_polar_density(drive_to_wp_state_t *state, double *xyt, double *polar_density)
{
    grid_map_t *gm = state->last_grid_map;

    int cx = gridmap_get_index_x(gm, xyt[0]);
    int cy = gridmap_get_index_y(gm, xyt[1]);

    for (int i = 0; i < state->polar_sections; i++) {
        polar_density[i] = 0;
    }

    double rads_to_section_i = state->polar_sections * (1.0 / (2 * M_PI));

    zarray_t *circle_lines = state->precomp_circle_lines;
    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++) {
            if (gm->data[(cy + y) * gm->width + cx + x] & GRID_FLAG_TRAVERSABLE) {
                continue;
            }
            double rads = state->precomp_radians[pixel_on];
            double magnitude = state->precomp_magnitudes[pixel_on];
            double enlargement_rad = state->precomp_enlargements[pixel_on];

            double rad_min = rads + (2 * M_PI) - enlargement_rad;
            int rad_min_i = (int)(rad_min * rads_to_section_i + 0.5) % state->polar_sections;
            double rad_max = rads + enlargement_rad;
            int rad_max_i = (int)(rad_max * rads_to_section_i + 0.5) % state->polar_sections;
            for (int j = rad_min_i; j <= rad_max_i; j++) {
                polar_density[j] += magnitude;
            }
        }
    }
}

void update_binary_polar_histogram(drive_to_wp_state_t *state, double *polar_density, uint8_t *binary_polar_histogram)
{
    for (int i = 0; i < state->polar_sections; i++) {
        double d = polar_density[i];
        if (d < state->polar_density_traversable) {
            binary_polar_histogram[i] = 0;
        } else if (d > state->polar_density_occupied) {
            binary_polar_histogram[i] = 1;
        }
    }
}

// wraps the value to [-sections/2, sections/2-1] modulo sections
int wrap_polar_sections(int value, int sections) {
    return (value + 3 * sections / 2) % sections - sections / 2;
}

void update_masked_polar_histogram(drive_to_wp_state_t *state, double *xyt, uint8_t *binary_polar_histogram, uint8_t *masked_binary_histogram)
{
    grid_map_t *gm = state->last_grid_map;

    int cx = gridmap_get_index_x(gm, xyt[0]);
    int cy = gridmap_get_index_y(gm, xyt[1]);

    double forward_rads = xyt[2];
    double min_turning_r = state->min_turning_r / gm->meters_per_pixel;

    int right_center_x = (int)(min_turning_r * sin(forward_rads) + 0.5);
    int right_center_y = (int)(min_turning_r * cos(forward_rads) + 0.5);
    int left_center_x = -right_center_x;
    int left_center_y = -right_center_y;

    double left_rad_limit = forward_rads + M_PI;
    double right_rad_limit = forward_rads - M_PI;

    double robot_rs = state->vehicle_width / 2 + state->planning_clearance;
    int dist_sq_limit = (int)(sq(min_turning_r + robot_rs) / gm->meters_per_pixel + 0.5);

    zarray_t *circle_lines = state->precomp_circle_lines;
    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++) {
            if (gm->data[(cy + y) * gm->width + cx + x] & GRID_FLAG_TRAVERSABLE) {
                continue;
            }
            double rads = state->precomp_radians[pixel_on];
            double delta_rads = mod2pi(rads - forward_rads);
            if (delta_rads > 0 && mod2pi(rads - left_rad_limit) < 0) {
                int left_dist_sq = isq(left_center_x - x) + isq(left_center_y - y);
                if (left_dist_sq < dist_sq_limit) {
                    left_rad_limit = rads;
                }
            }
            if (delta_rads <= 0 && mod2pi(rads - right_rad_limit) > 0) {
                int right_dist_sq = isq(right_center_x - x) + isq(right_center_y - y);
                if (right_dist_sq < dist_sq_limit) {
                    right_rad_limit = rads;
                }
            }
        }
    }

    double rads_to_section_i = state->polar_sections * (1.0 / (2 * M_PI));
    int left_limit_i = (int)(mod2pi_positive(left_rad_limit) * rads_to_section_i + 0.5);
    int right_limit_i = (int)(mod2pi_positive(right_rad_limit) * rads_to_section_i + 0.5);

    int sections = state->polar_sections;
    for (int i = 0; i < sections; i++) {
        if (wrap_polar_sections(i - right_limit_i, sections) > 0 && // essentially i > right_limit_i with bounds wrapped
            wrap_polar_sections(i - left_limit_i, sections) < 0) {
            masked_binary_histogram[i] = binary_polar_histogram[i];
        } else {
            masked_binary_histogram[i] = 1;
        }
    }
}

int section_i_difference(int a, int b, int n)
{
    return (a - b + n) % n;
}

float primary_direction_cost(vfh_plus_t *vfh)
{
    drive_to_wp_state_t *state = vfh->state;
    int direction_i = vfh->direction_i;
    int n = state->polar_sections;
    double rads_to_section_i = n * (1.0 / (2 * M_PI));

    int target_direction_i = (int)(state->target_direction * rads_to_section_i + 0.5);
    target_direction_i = (target_direction_i + n) % n;

    int current_direction_i = (int)(state->xyt[2] * rads_to_section_i + 0.5);
    current_direction_i = (current_direction_i + n) % n;

    double cost = state->cost_goal_oriented * section_i_difference(direction_i, target_direction_i, n) +
                  state->cost_smooth_path * section_i_difference(direction_i, current_direction_i, n) +
                  state->cost_smooth_commands * section_i_difference(direction_i, state->last_target_direction_i, n);
    return (float)cost;
}

float projected_direction_cost(vfh_plus_t *vfh)
{
    drive_to_wp_state_t *state = vfh->state;
    int direction_i = vfh->direction_i;
    int n = state->polar_sections;
    double rads_to_section_i = n * (1.0 / (2 * M_PI));

    int target_direction_i = (int)(state->target_direction * rads_to_section_i + 0.5);
    target_direction_i = (target_direction_i + n) % n;

    int current_direction_i = (int)(state->xyt[2] * rads_to_section_i + 0.5);
    current_direction_i = (current_direction_i + n) % n;

    double cost = state->cost_goal_oriented * section_i_difference(direction_i, target_direction_i, n) +
                  state->cost_smooth_path * section_i_difference(direction_i, current_direction_i, n) +
                  state->cost_smooth_commands * section_i_difference(direction_i, state->last_target_direction_i, n);
    return (float)cost;
}

vfh_plus_t make_vfh_plus_for(drive_to_wp_state_t *state, double *xyt, int direction_i)
{
    int n = state->polar_sections;

    vfh_plus_t vfh;
    vfh.state = state;
    vfh.direction_i = (direction_i + n) % n;
    vfh.next_vfh_pluses = NULL;

    double direction = vfh.direction_i * (2 * M_PI) / n;
    double d_theta = mod2pi(direction - xyt[2]);
    double r = state->min_turning_r;
    double d = state->step_distance;

    double t = fabs(d_theta);
    double t_sign = sgn(d_theta);

    if (d <= t * r) {
        // we can't make the whole turn in this distance
        // we stay on the minimum turning radius circle
        vfh.xyt[0] = xyt[0] + r * sin(t);
        vfh.xyt[1] = xyt[1] + t_sign * r * (1.0 - cos(t));
        vfh.xyt[2] = xyt[2] + t_sign * d / r;
    } else {
        vfh.xyt[0] = xyt[0] + r * sin(t) + (d - t * r) * cos(t);
        vfh.xyt[1] = xyt[1] + t_sign * (r * (1.0 - cos(t)) + (d - t * r) * sin(t));
        vfh.xyt[2] = direction;
    }

    return vfh;
}

zarray_t *vfh_plus_next_at(drive_to_wp_state_t *state, double *xyt)
{
    int n = state->polar_sections;
    double polar_density[n];
    uint8_t binary_polar_histogram[n];
    uint8_t masked_binary_histogram[n];

    update_polar_density(state, xyt, polar_density);
    update_binary_polar_histogram(state, polar_density, binary_polar_histogram);
    update_masked_polar_histogram(state, xyt, binary_polar_histogram, masked_binary_histogram);

    zarray_t *next_vfh_pluses = zarray_create(sizeof(vfh_plus_t));

    double rads_to_section_i = n / (2 * M_PI);

    int target_direction_i = (int)(state->target_direction * rads_to_section_i + 0.5);
    target_direction_i = (target_direction_i + n) % n;

    for (int start_i = 0; start_i < n; start_i++) {
        if (masked_binary_histogram[start_i]) {
            continue;
        }
        // find boundaries of the opening
        int left_i = start_i;
        int right_i = start_i;
        while (masked_binary_histogram[(right_i + n - 1) % n] == 0) {
            right_i = (right_i + n - 1) % n;
        }
        while (masked_binary_histogram[(left_i + 1) % n] == 0) {
            left_i = (left_i + 1) % n;
        }
        int opening_size = (left_i - right_i + n) % n + 1;
        if (opening_size >= state->wide_opening_size) {
            // left-side opening
            int left_dir_i = left_i - state->wide_opening_size / 2 + 1;
            vfh_plus_t new_vfh = make_vfh_plus_for(state, xyt, left_dir_i);
            zarray_add(next_vfh_pluses, &new_vfh);

            int right_dir_i = right_i + state->wide_opening_size / 2 - 1;
            new_vfh = make_vfh_plus_for(state, xyt, right_dir_i);
            zarray_add(next_vfh_pluses, &new_vfh);

            // is target_direction_i between right_i and left_i
            if (((target_direction_i - right_i + n) % n) <= ((left_i - right_i + n) % n)) {
                new_vfh = make_vfh_plus_for(state, xyt, target_direction_i);
                zarray_add(next_vfh_pluses, &new_vfh);
            }
        } else {
            // small opening, just add the center through it.
            int center_i = right_i + (opening_size - 1) / 2;
            vfh_plus_t new_vfh = make_vfh_plus_for(state, xyt, center_i);
            zarray_add(next_vfh_pluses, &new_vfh);
        }
    }

    return next_vfh_pluses;
}

bool is_goal(gen_search_node_t *node)
{
    vfh_plus_t *vfh = (vfh_plus_t*)node->state;
    drive_to_wp_state_t *state = vfh->state;
    return node->depth == state->goal_depth;
}

float step_cost(gen_search_node_t *node, int32_t action)
{
    vfh_plus_t *vfh = (vfh_plus_t*)node->state;
    if (node->depth <= 1) {
        return primary_direction_cost(vfh);
    } else {
        return projected_direction_cost(vfh);
    }
}

float heuristic_cost(gen_search_node_t *node)
{
    return step_cost(node, 0);
}

float ordering_cost(gen_search_node_t *node)
{
    return node->path_cost + heuristic_cost(node);
}

void *expand_state(void *state)
{
    vfh_plus_t *vfh = (vfh_plus_t*)state;
    drive_to_wp_state_t *s = vfh->state;
    if (vfh->next_vfh_pluses) {
        // already expanded?
        printf("Node already expanded!?\n");
    } else {
        vfh->next_vfh_pluses = vfh_plus_next_at(s, vfh->xyt);
    }

    expansion_t *expansion = calloc(1, sizeof(expansion_t));
    expansion->vfh = vfh;
    expansion->i = 0;

    return expansion;
}

bool next_new_state(void *expansion, void **new_state, int32_t *new_action)
{
    expansion_t *e = (expansion_t*)expansion;
    if (e->i >= zarray_size(e->vfh->next_vfh_pluses)) {
        free(e);
        return false;
    }

    *new_action = e->i;
    zarray_get_volatile(e->vfh->next_vfh_pluses, e->i, new_state);
    e->i++;

    return true;
}

void vfh_star_update(drive_to_wp_state_t *state)
{
    if (!state->last_grid_map) {
        return;
    }
    if (!state->vfh_has_inited) {
        delayed_initialize_vfh_star(state);
        state->vfh_has_inited = true;
    }

    vfh_plus_t initial_vfh;
    initial_vfh.state = state;
    initial_vfh.direction_i = -1;
    initial_vfh.next_vfh_pluses = NULL;
    memcpy(initial_vfh.xyt, state->xyt, sizeof(initial_vfh.xyt));

    // set up an A* problem
    general_search_problem_t p;
    p.initial_state = &initial_vfh;
    p.is_goal = is_goal;
    p.step_cost = step_cost;
    p.ordering_cost = ordering_cost;
    p.expand_state = expand_state;
    p.next_new_state = next_new_state;
    populate_with_unordered_set(&p);
    populate_with_priority_queue(&p);

    gen_search_node_t *result = tree_search(&p);
    gen_search_node_t *parent = result;
    // we just need the top level choice of direction!
    while (parent->depth > 1) {
        parent = parent->parent;
    }

    vfh_plus_t *vfh = (vfh_plus_t*)parent->state;
    double section_i_to_rads = (2 * M_PI) / state->polar_sections;
    state->chosen_direction = vfh->direction_i * section_i_to_rads;

    render_vfh_star(state, result);

    general_search_result_destroy(result);
}
