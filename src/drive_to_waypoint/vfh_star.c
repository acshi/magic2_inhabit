#include "vfh_star.h"
#include "gui.h"
#include "common/general_search.h"

#define PREFX "drive_to_wp.vfh_star."

typedef struct expansion {
    vfh_plus_t *vfh;
    int i;
} expansion_t;

static void circle_add_lines(zarray_t *lines, int cx, int cy, int x, int y)
{
    int line_top[3] = {cx - x, cx + x, cy + y};
    zarray_add(lines, line_top);

    if (y != 0) {
        int line_bottom[3] = {cx - x, cx + x, cy - y};
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
    state->polar_sections = config_require_int(config, PREFX "polar_sections");
    require_value_nonnegative(state->polar_sections, PREFX "polar_sections");

    state->wide_opening_size = config_require_int(config, PREFX "wide_opening_size");
    require_value_nonnegative(state->wide_opening_size, PREFX "wide_opening_size");

    state->active_diameter = config_require_double(config, PREFX "active_diameter");
    require_value_nonnegative(state->active_diameter, PREFX "active_diameter");

    state->max_magnitude = config_require_double(config, PREFX "max_magnitude");
    require_value_nonnegative(state->max_magnitude, PREFX "max_magnitude");

    state->planning_clearance = config_require_double(config, PREFX "planning_clearance");
    require_value_nonnegative(state->planning_clearance, PREFX "planning_clearance");

    state->polar_density_traversable = config_require_double(config, PREFX "polar_density_traversable");
    require_value_nonnegative(state->polar_density_traversable, PREFX "polar_density_traversable");

    state->polar_density_occupied = config_require_double(config, PREFX "polar_density_occupied");
    require_value_nonnegative(state->polar_density_occupied, PREFX "polar_density_occupied");

    state->step_distance = config_require_double(config, PREFX "step_distance");
    require_value_nonnegative(state->step_distance, PREFX "step_distance");

    state->goal_depth = config_require_int(config, PREFX "goal_depth");
    require_value_nonnegative(state->goal_depth, PREFX "goal_depth");

    state->discount_factor = config_require_double(config, PREFX "discount_factor");
    require_value_nonnegative(state->discount_factor, PREFX "discount_factor");

    state->cost_goal_oriented = config_require_int(config, PREFX "cost_goal_oriented");
    require_value_nonnegative(state->cost_goal_oriented, PREFX "cost_goal_oriented");

    state->cost_smooth_path = config_require_int(config, PREFX "cost_smooth_path");
    require_value_nonnegative(state->cost_smooth_path, PREFX "cost_smooth_path");

    state->cost_smooth_commands = config_require_int(config, PREFX "cost_smooth_commands");
    require_value_nonnegative(state->cost_smooth_commands, PREFX "cost_smooth_commands");

    state->cost_proj_goal_oriented = config_require_int(config, PREFX "cost_proj_goal_oriented");
    require_value_nonnegative(state->cost_proj_goal_oriented, PREFX "cost_proj_goal_oriented");

    state->cost_proj_smooth_path = config_require_int(config, PREFX "cost_proj_smooth_path");
    require_value_nonnegative(state->cost_proj_smooth_path, PREFX "cost_proj_smooth_path");

    state->cost_proj_smooth_commands = config_require_int(config, PREFX "cost_proj_smooth_commands");
    require_value_nonnegative(state->cost_proj_smooth_commands, PREFX "cost_proj_smooth_commands");

    state->chosen_directions_i = calloc(state->goal_depth, sizeof(*state->chosen_directions_i));
}

void delayed_initialize_vfh_star(drive_to_wp_state_t *state)
{
    grid_map_t *gm = state->last_grid_map;

    int r = (int)(state->active_diameter / 2 / gm->meters_per_pixel + 0.5);

    // double robot_rs = state->vehicle_diam / 2 + state->planning_clearance;
    double scale_b = (state->max_magnitude - 1) / sq((state->active_diameter - 0) / 2);

    zarray_t *circle_lines = rasterize_circle_lines(0, 0, r);
    state->precomp_circle_lines = circle_lines;

    int pixels_in_circle = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        pixels_in_circle += line[1] - line[0] + 1;
    }

    state->precomp_radians = calloc(pixels_in_circle, sizeof(double));
    state->precomp_invdist = calloc(pixels_in_circle, sizeof(double));
    // state->precomp_enlargements = calloc(pixels_in_circle, sizeof(double));
    state->precomp_magnitudes = calloc(pixels_in_circle, sizeof(float));
    state->cached_enlargements = calloc(pixels_in_circle, sizeof(double));

    double sq_pixels_to_meters = (gm->meters_per_pixel * gm->meters_per_pixel);

    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++) {
            double rads = atan2(y, x);
            double dist_sq = (x * x + y * y) * sq_pixels_to_meters;
            double inv_dist = 1.0 / sqrt(dist_sq);
            double magnitude = (state->max_magnitude - scale_b * dist_sq);
            // double enlargement_rad = asin(min(1, robot_rs / dist));
            state->precomp_radians[pixel_on] = rads;
            state->precomp_invdist[pixel_on] = inv_dist;
            // state->precomp_enlargements[pixel_on] = enlargement_rad;
            state->precomp_magnitudes[pixel_on] = (float)magnitude;
        }
    }
}

void update_cached_enlargements(drive_to_wp_state_t *state)
{
    double robot_rs = state->vehicle_diam / 2 + state->planning_clearance;
    if (state->cached_enlargement_robot_rs == robot_rs) {
        return;
    }

    zarray_t *circle_lines = state->precomp_circle_lines;
    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        for (int x = line[0]; x <= line[1]; x++, pixel_on++) {
            double invdist = state->precomp_invdist[pixel_on];
            double enlargement_rad = asin(min(1, robot_rs * invdist));
            state->cached_enlargements[pixel_on] = enlargement_rad;
        }
    }

    state->cached_enlargement_robot_rs = robot_rs;
}

void update_polar_density(drive_to_wp_state_t *state, vfh_plus_t *vfh, float *polar_density)
{
    grid_map_t *gm = state->last_grid_map;
    double *xyt = vfh->xyt;

    int cx = gridmap_get_index_x(gm, xyt[0]);
    int cy = gridmap_get_index_y(gm, xyt[1]);

    for (int i = 0; i < state->polar_sections; i++) {
        polar_density[i] = 0;
    }

    int n = state->polar_sections;
    double rads_to_section_i = n * (1.0 / (2 * M_PI));

    int star_active_dist_sq = (int)sq(vfh->star_active_d / 2 / gm->meters_per_pixel);

    update_cached_enlargements(state);

    zarray_t *circle_lines = state->precomp_circle_lines;
    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        uint8_t *gm_data = &gm->data[(cy + y) * gm->width + cx + line[0]];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++, gm_data++) {
            if (*gm_data & GRID_FLAG_TRAVERSABLE) {
                continue;
            }
            if (vfh->star_active_d < state->active_diameter) {
                // use a smaller effective circle...
                if (x * x + y * y > star_active_dist_sq) {
                    continue;
                }
            }
            double rads = state->precomp_radians[pixel_on];
            float magnitude = state->precomp_magnitudes[pixel_on];
            double enlargement_rad = state->cached_enlargements[pixel_on];

            double rad_min = rads - enlargement_rad;
            double rad_max = rads + enlargement_rad;

            // rad_max_i is strictly >= than rad_min_i
            int rad_min_i = (int)(rad_min * rads_to_section_i + 0.5);
            int rad_max_i = (int)(rad_max * rads_to_section_i + 0.5);

            // but these are the actual indices
            int first_i = (rad_min_i + n) % n;
            int last_i = (rad_max_i + n) % n;

            if (first_i < last_i) {
                for (int j = first_i; j <= last_i; j++) {
                    polar_density[j] += magnitude;
                }
            } else {
                for (int j = first_i; j < n; j++) {
                    polar_density[j] += magnitude;
                }
                for (int j = 0; j <= last_i; j++) {
                    polar_density[j] += magnitude;
                }
            }
        }
    }
}

void update_binary_polar_histogram(drive_to_wp_state_t *state, float *polar_density, uint8_t *binary_polar_histogram, bool debug)
{
    for (int i = 0; i < state->polar_sections; i++) {
        float d = polar_density[i];
        if (d < state->polar_density_traversable) {
            binary_polar_histogram[i] = 0;
            if (debug) {
                printf("(%d: %4.1f) ", i, d);
            }
        } else if (d > state->polar_density_occupied) {
            binary_polar_histogram[i] = 1;
        }
    }
    if (debug) {
        printf("\n");
    }
}

// wraps the value to [-sections/2, sections/2-1] modulo sections
int wrap_polar_sections(int value, int sections)
{
    return (value + 3 * sections / 2) % sections - sections / 2;
}

// quick mod2pi, for when only ever off by at most 2pi
static inline double q_mod2pi(double a)
{
    if (a < -M_PI) {
        return a + 2 * M_PI;
    } else if (a >= M_PI) {
        return a - 2 * M_PI;
    }
    return a;
}

void update_masked_polar_histogram(drive_to_wp_state_t *state, vfh_plus_t *vfh, uint8_t *binary_polar_histogram, uint8_t *masked_binary_histogram)
{
    int n = state->polar_sections;
    grid_map_t *gm = state->last_grid_map;
    double *xyt = vfh->xyt;

    int cx = gridmap_get_index_x(gm, xyt[0]);
    int cy = gridmap_get_index_y(gm, xyt[1]);

    double forward_rads = xyt[2];
    double min_turning_r = state->min_turning_r / gm->meters_per_pixel;

    int right_center_x = (int)(min_turning_r * sin(forward_rads) + 0.5);
    int right_center_y = (int)(-min_turning_r * cos(forward_rads) + 0.5);
    int left_center_x = -right_center_x;
    int left_center_y = -right_center_y;

    double left_rad_limit = forward_rads + M_PI;
    double right_rad_limit = forward_rads - M_PI + (2 * M_PI / n);

    double robot_rs = state->vehicle_diam / 2 + state->planning_clearance;
    int dist_sq_limit = (int)(sq((state->min_turning_r + robot_rs) / gm->meters_per_pixel) + 0.5);

    int star_active_dist_sq = (int)sq(vfh->star_active_d / 2 / gm->meters_per_pixel);

    zarray_t *circle_lines = state->precomp_circle_lines;
    int pixel_on = 0;
    for (int i = 0; i < zarray_size(circle_lines); i++) {
        int line[3];
        zarray_get(circle_lines, i, line);
        int y = line[2];
        uint8_t *gm_data = &gm->data[(cy + y) * gm->width + cx + line[0]];
        for (int x = line[0]; x <= line[1]; x++, pixel_on++, gm_data++) {
            if (*gm_data & GRID_FLAG_TRAVERSABLE) {
                continue;
            }
            if (vfh->star_active_d < state->active_diameter) {
                // use a smaller effective circle...
                if (x * x + y * y > star_active_dist_sq) {
                    continue;
                }
            }
            double rads = state->precomp_radians[pixel_on];
            double delta_rads = q_mod2pi(rads - forward_rads);
            if (delta_rads > 0 && q_mod2pi(rads - left_rad_limit) < 0) {
                int left_dist_sq = isq(left_center_x - x) + isq(left_center_y - y);
                if (left_dist_sq < dist_sq_limit) {
                    left_rad_limit = rads;
                }
            }
            if (delta_rads <= 0 && q_mod2pi(rads - right_rad_limit) > 0) {
                int right_dist_sq = isq(right_center_x - x) + isq(right_center_y - y);
                if (right_dist_sq < dist_sq_limit) {
                    right_rad_limit = rads;
                }
            }
        }
    }

    double rads_to_section_i = n * (1.0 / (2 * M_PI));
    int left_limit_i = (int)(mod2pi_positive(left_rad_limit) * rads_to_section_i + 0.5);
    int right_limit_i = (int)(mod2pi_positive(right_rad_limit) * rads_to_section_i + 0.5);

    for (int i = 0; i < n; i++) {
        // is i between the two limits
        if (((i - right_limit_i + n) % n) <= ((left_limit_i - right_limit_i + n) % n)) {
            masked_binary_histogram[i] = binary_polar_histogram[i];
        } else {
            masked_binary_histogram[i] = 1;
        }
    }
}

void calculate_derived_vfh_properties(drive_to_wp_state_t *state, vfh_plus_t *vfh)
{
    // limit step distance to needed distance to achieve goal
    // and corresponding lower the active diameter
    double dist_dx = state->target_x - vfh->xyt[0];
    double dist_dy = state->target_y - vfh->xyt[1];
    double dist_left = sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
    double step = min(dist_left, state->step_distance);
    vfh->star_active_d = state->active_diameter - 2 * (state->step_distance - step);
    vfh->star_step_dist = step;
    vfh->target_dir = atan2(dist_dy, dist_dx);
}

vfh_plus_t make_termination_vfh_for(drive_to_wp_state_t *state, vfh_plus_t *prior_vfh)
{
    int n = state->polar_sections;

    vfh_plus_t vfh = *prior_vfh;
    vfh.is_early_termination = true;
    vfh.parent = prior_vfh;

    size_t hist_size = n * sizeof(*vfh.binary_histogram);
    vfh.binary_histogram = malloc(hist_size);
    vfh.masked_histogram = malloc(hist_size);
    memcpy(vfh.binary_histogram, prior_vfh->binary_histogram, hist_size);
    memcpy(vfh.masked_histogram, prior_vfh->masked_histogram, hist_size);

    vfh.next_vfh_pluses = NULL;
    vfh.step_cost = 0;

    // printf("Made termination vfh at depth %d, direction %d\n", vfh.depth, vfh.direction_i);
    return vfh;
}

vfh_plus_t make_vfh_plus_for(drive_to_wp_state_t *state, vfh_plus_t *prior_vfh, int direction_i)
{
    int n = state->polar_sections;
    double *xyt = prior_vfh->xyt;

    vfh_plus_t vfh = { 0 };
    vfh.state = state;
    vfh.parent = prior_vfh;
    vfh.direction_i = (direction_i + n) % n;
    vfh.next_vfh_pluses = NULL;
    vfh.binary_histogram = NULL;
    vfh.masked_histogram = NULL;
    vfh.depth = prior_vfh->depth + 1;
    vfh.min_turning_r = state->min_turning_r;

    double direction = vfh.direction_i * (2 * M_PI) / n;
    double d_theta = mod2pi(direction - xyt[2]);
    double r = state->min_turning_r;

    // limit step distance to needed distance to achieve goal
    double d = prior_vfh->star_step_dist;

    double t = fabs(d_theta);
    double t_sign = sgn(d_theta);

    // "local" dx dy with the assumption the robot's theta is 0.
    double ldx;
    double ldy;
    if (d <= t * r) {
        // we can't make the whole turn in this distance
        // we stay on the minimum turning radius circle
        // limit t to just the part we can complete in distance d
        double t2 = d / r * t_sign;
        ldx = r * sin(fabs(t2));
        ldy = t_sign * r * (1.0 - cos(t2));
        vfh.xyt[2] = mod2pi(xyt[2] + t2);
    } else {
        ldx = r * sin(fabs(t)) + (d - t * r) * cos(t);
        ldy = t_sign * (r * (1.0 - cos(t)) + (d - t * r) * sin(t));
        vfh.xyt[2] = direction;
        //printf("for xy: [%.2f, %.2f], d*cos(t): %.2f, -d*sin(t): %.2f, direction: %.3f\n", vfh.xyt[0], vfh.xyt[1], d * cos(t), -d * sin(t), direction);
    }
    // make actual dx dy from local ones and robot's orientation
    double sin_robot_th = sin(xyt[2]);
    double cos_robot_th = cos(xyt[2]);
    double dx = ldx * cos_robot_th - ldy * sin_robot_th;
    double dy = ldx * sin_robot_th + ldy * cos_robot_th;
    vfh.xyt[0] = xyt[0] + dx;
    vfh.xyt[1] = xyt[1] + dy;
    vfh.effective_dir = atan2(dy, dx);

    calculate_derived_vfh_properties(state, &vfh);
    // printf("Made vfh at depth %d, direction %d\n", vfh.depth, vfh.direction_i);
    return vfh;
}

void vfh_plus_update_histograms(drive_to_wp_state_t *state, vfh_plus_t *vfh)
{
    int n = state->polar_sections;
    float polar_density[n];

    if (!vfh->binary_histogram) {
        vfh->binary_histogram = calloc(n, sizeof(*vfh->binary_histogram));
    }
    if (!vfh->masked_histogram) {
        vfh->masked_histogram = calloc(n, sizeof(*vfh->masked_histogram));
    }

    update_polar_density(state, vfh, polar_density);
    update_binary_polar_histogram(state, polar_density, vfh->binary_histogram, false);
    memcpy(vfh->masked_histogram, vfh->binary_histogram, n * sizeof(*vfh->masked_histogram));
    // update_masked_polar_histogram(state, vfh, vfh->binary_histogram, vfh->masked_histogram);
}

bool section_between_i(int left_i, int right_i, int i, int n)
{
    return ((i - right_i + n) % n) <= ((left_i - right_i + n) % n);
}

zarray_t *vfh_plus_next_from(drive_to_wp_state_t *state, vfh_plus_t *vfh)
{
    vfh_plus_update_histograms(state, vfh);
    zarray_t *next_vfh_pluses = zarray_create(sizeof(vfh_plus_t));

    int n = state->polar_sections;
    double rads_to_section_i = n / (2 * M_PI);

    int target_direction_i = (int)(vfh->target_dir * rads_to_section_i + 0.5);
    target_direction_i = (target_direction_i + n) % n;

    int first_right_i = n;
    for (int start_i = 0; start_i < first_right_i; start_i++) {
        if (vfh->masked_histogram[start_i]) {
            continue;
        }
        // find boundaries of the opening
        int left_i = start_i;
        int right_i = start_i;
        while (vfh->masked_histogram[(right_i + n - 1) % n] == 0) {
            right_i = (right_i + n - 1) % n;
            if (right_i == start_i) {
                break;
            }
        }
        while (vfh->masked_histogram[(left_i + 1) % n] == 0) {
            left_i = (left_i + 1) % n;
            if (left_i == start_i) {
                break;
            }
        }
        int opening_size;
        if (left_i == right_i && left_i == start_i &&
            vfh->masked_histogram[(start_i + 1) % n] == 0) {
            // entire histogram is clear
            opening_size = n;
            left_i = n - 1;
            right_i = 0;
        } else {
            opening_size = (left_i - right_i + n) % n + 1;
        }

        if (opening_size >= state->wide_opening_size) {
            bool target_between_limits = section_between_i(left_i, right_i, target_direction_i, n);

            if (opening_size <= 3) {
                // special case for small openings when allowed
                // consider all of our limited options here...
                vfh_plus_t new_vfh = make_vfh_plus_for(state, vfh, left_i);
                zarray_add(next_vfh_pluses, &new_vfh);

                new_vfh = make_vfh_plus_for(state, vfh, right_i);
                zarray_add(next_vfh_pluses, &new_vfh);

                if (target_between_limits &&
                        target_direction_i != left_i &&
                        target_direction_i != right_i) {
                    new_vfh = make_vfh_plus_for(state, vfh, target_direction_i);
                    zarray_add(next_vfh_pluses, &new_vfh);
                }
            } else {
                int left_dir_i = left_i + state->wide_opening_size / 2 - 1;
                vfh_plus_t new_vfh = make_vfh_plus_for(state, vfh, left_dir_i);
                zarray_add(next_vfh_pluses, &new_vfh);

                int right_dir_i = right_i - state->wide_opening_size / 2 + 1;
                new_vfh = make_vfh_plus_for(state, vfh, right_dir_i);
                zarray_add(next_vfh_pluses, &new_vfh);

                // the range over which we will add options at high density
                int option_range = 8;

                if (target_between_limits) {
                    if (target_direction_i != left_dir_i &&
                            target_direction_i != right_dir_i) {
                        new_vfh = make_vfh_plus_for(state, vfh, target_direction_i);
                        zarray_add(next_vfh_pluses, &new_vfh);
                    }

                    // add new states at every x through the opening'
                    // limited to some amount from the center
                    int option_start_i;
                    int option_end_i;
                    if (opening_size <= option_range) {
                        option_start_i = 0;
                        option_end_i = opening_size;
                    } else {
                        int loss_i = (opening_size - option_range) / 2;
                        option_start_i = loss_i;
                        option_end_i = opening_size - loss_i;
                    }
                    for (int i = option_start_i; i < option_end_i; i += 2) {
                        int option_dir_i = (right_i + i) % n;
                        if (option_dir_i == target_direction_i ||
                            option_dir_i == left_dir_i ||
                            option_dir_i == right_dir_i) {
                            continue;
                        }
                        new_vfh = make_vfh_plus_for(state, vfh, option_dir_i);
                        zarray_add(next_vfh_pluses, &new_vfh);
                    }
                } else if (section_between_i(left_i + option_range, left_i, target_direction_i, n)) {
                    // The target is just to the left of the left limit
                    // We consider many of these left-most options
                    int option_start_i = 0;
                    int option_end_i = min(opening_size, option_range);
                    for (int i = option_start_i; i < option_end_i; i += 2) {
                        int option_dir_i = (left_i - i + n) % n;
                        if (option_dir_i == left_dir_i ||
                            option_dir_i == right_dir_i) {
                            continue;
                        }
                        new_vfh = make_vfh_plus_for(state, vfh, option_dir_i);
                        zarray_add(next_vfh_pluses, &new_vfh);
                    }
                }
                else if (section_between_i(right_i, right_i - option_range, target_direction_i, n)) {
                   // Same for right of the right limit
                   int option_start_i = 0;
                   int option_end_i = min(opening_size, option_range);
                   for (int i = option_start_i; i < option_end_i; i += 2) {
                       int option_dir_i = (right_i + i) % n;
                       if (option_dir_i == left_dir_i ||
                           option_dir_i == right_dir_i) {
                           continue;
                       }
                       new_vfh = make_vfh_plus_for(state, vfh, option_dir_i);
                       zarray_add(next_vfh_pluses, &new_vfh);
                   }
               }
            }
        } else {
            // small opening, just add the center through it.
            int center_i = right_i + (opening_size - 1) / 2;
            vfh_plus_t new_vfh = make_vfh_plus_for(state, vfh, center_i);
            zarray_add(next_vfh_pluses, &new_vfh);
        }
        if (start_i == 0) {
            first_right_i = right_i;
        }
        start_i = left_i;
    }

    // Add a final result equivalent to the current VFH
    // but marked as a termination state (is_goal = true)
    vfh_plus_t terminated_vfh = make_termination_vfh_for(state, vfh);
    zarray_add(next_vfh_pluses, &terminated_vfh);

    return next_vfh_pluses;
}

int section_i_diff(int a, int b, int n)
{
    return abs((a - b + n * 3 / 2) % n - n / 2);
}

float direction_cost(vfh_plus_t *vfh, double target_dir, double *prior_xyt, int depth, bool debug)
{
    drive_to_wp_state_t *state = vfh->state;
    int dir_i = vfh->direction_i;
    int n = state->polar_sections;
    double rads_to_section_i = n * (1.0 / (2 * M_PI));

    int target_dir_i = (int)(target_dir * rads_to_section_i + 0.5);
    target_dir_i = (target_dir_i + n) % n;

    int current_dir_i = (int)(prior_xyt[2] * rads_to_section_i + 0.5);
    current_dir_i = (current_dir_i + n) % n;

    int goal_diff = section_i_diff(dir_i, target_dir_i, n);

    if (depth > 1) {
        int effective_dir_i = (int)(vfh->effective_dir * rads_to_section_i + 0.5);
        effective_dir_i = (effective_dir_i + n) % n;

        goal_diff = max(goal_diff, section_i_diff(effective_dir_i, target_dir_i, n));
    }

    int cost_1 = (depth > 1) ? state->cost_proj_goal_oriented : state->cost_goal_oriented;
    int cost_2 = (depth > 1) ? state->cost_proj_smooth_path: state->cost_smooth_path;
    int cost_3 = (depth > 1) ? state->cost_proj_smooth_commands : state->cost_smooth_commands;

    int goal_cost = cost_1 * goal_diff;
    int path_cost = cost_2 * section_i_diff(dir_i, current_dir_i, n);
    int smooth_cost = cost_3 * section_i_diff(dir_i, state->chosen_directions_i[depth - 1], n);
    double cost = goal_cost + path_cost + smooth_cost;

    cost *= pow(state->discount_factor, depth - 1);

    if (debug) {
        printf("min_r: %.1f depth: %d tdir_i: %2d dir_i: %2d goal_c: %2d, path_c: %2d, smooth_c: %2d, end_cost: %3.1f\n",
                vfh->min_turning_r, vfh->depth, target_dir_i, dir_i, goal_cost, path_cost, smooth_cost, cost);
    }

    return (float)cost;
}

bool is_goal(gen_search_node_t *node)
{
    vfh_plus_t *vfh = (vfh_plus_t*)node->state;
    drive_to_wp_state_t *state = vfh->state;
    return node->depth == state->star_depth || vfh->is_early_termination;
}

// final evaluation of end location as compared to desired goal point
float goal_result_cost(vfh_plus_t *vfh)
{
    drive_to_wp_state_t *state = vfh->state;
    double initial_dist = sqrt(sq(state->xyt[0] - state->target_x) + sq(state->xyt[1] - state->target_y));
    double end_dist = sqrt(sq(vfh->xyt[0] - state->target_x) + sq(vfh->xyt[1] - state->target_y));

    double result_cost = 0;

    double progress = initial_dist - end_dist;
    if (progress > 0) {
        // assign reward (negative value) based on how much closer we are
        result_cost += -progress;
    }
    // if (progress < 0) {
    //     // farther away than when we started. penalize this highly!
    //     result_cost += 1000000;
    // } else if (progress < vfh->star_step_dist * vfh->depth * 0.5) {
    //     // a good result must achieve say...
    //     // 50% of the possible distance reduction. penalize otherwise
    //     result_cost += 50000;
    // } else {
    //     // assign reward (negative value) based on how much closer we are
    //     result_cost += -progress;
    // }

    int turn_count = 0;
    vfh_plus_t *parent = vfh;
    int n = state->polar_sections;

    double parent_dist = sqrt(sq(vfh->xyt[0] - state->target_x) + sq(vfh->xyt[1] - state->target_y));
    while (parent->parent) {
        vfh_plus_t *next_parent = parent->parent;
        if (next_parent->direction_i != -1) {
            turn_count += section_i_diff(parent->direction_i, next_parent->direction_i, n);
        }

        double next_parent_dist = sqrt(sq(next_parent->xyt[0] - state->target_x) + sq(next_parent->xyt[1] - state->target_y));
        double step_progress = next_parent_dist - parent_dist;
        if (step_progress < 0) {
            // farther away than the last vfh! We got worse, bad!
            result_cost += 1000000;
        }
        parent_dist = next_parent_dist;

        parent = next_parent;
    }

    // penalize an excessive amount of change in chosen direction, which
    // indicates back and forth motion.
    if (turn_count >= n / 4) {
        result_cost += 10000;
    }

    // small penalty for early termination of the forward prediction
    if (vfh->is_early_termination) {
        result_cost += 1000 * (state->goal_depth - vfh->depth);
    }

    return (float)result_cost;
}

float step_cost(gen_search_node_t *node, int32_t action)
{
    // initial state has no cost
    if (node->depth == 0) {
        return 0;
    }

    vfh_plus_t *vfh = (vfh_plus_t*)node->state;
    if (vfh->step_cost != 0) {
        return vfh->step_cost;
    }

    vfh_plus_t *parent_vfh = (vfh_plus_t*)node->parent->state;
    double *parent_xyt = parent_vfh->xyt;
    double target_dir = parent_vfh->target_dir;

    bool debug = false;//node->depth == 1;
    if (vfh->is_early_termination) {
        // the early termination is equivalent to its parent but is a goal node
        vfh->step_cost = 0; // this cost already in the 'parent'
    } else {
        vfh->step_cost = direction_cost(vfh, target_dir, parent_xyt, node->depth, debug);
    }

    if (node->depth == vfh->state->star_depth || vfh->is_early_termination) {
        float result_cost = goal_result_cost(vfh);
        // printf("Total cost: %4.2f Goal cost: %4.2f\n", node->parent->path_cost + vfh->step_cost, result_cost);
        vfh->step_cost += result_cost;
    } else {
        // printf("Non-goal node cost: %4.2f\n", node->parent->path_cost + vfh->step_cost);
    }

    return vfh->step_cost;
}

float heuristic_cost(gen_search_node_t *node)
{
    vfh_plus_t *vfh = (vfh_plus_t*)node->state;
    drive_to_wp_state_t *state = vfh->state;
    double target_dir = vfh->target_dir;
    int depth = vfh->depth;

    vfh_plus_t *parent_vfh = vfh->parent;
    double *prior_xyt;
    if (parent_vfh) {
        prior_xyt = parent_vfh->xyt;
    } else {
        prior_xyt = state->xyt;
    }

    int n = state->polar_sections;
    double rads_to_section_i = n * (1.0 / (2 * M_PI));

    int target_dir_i = (int)(target_dir * rads_to_section_i + 0.5);
    target_dir_i = (target_dir_i + n) % n;

    int current_dir_i = (int)(prior_xyt[2] * rads_to_section_i + 0.5);
    current_dir_i = (current_dir_i + n) % n;

    int cost_2 = state->cost_proj_smooth_path;
    int cost_3 = state->cost_proj_smooth_commands;

    int path_cost = cost_2 * section_i_diff(target_dir_i, current_dir_i, n);
    int smooth_cost = cost_3 * section_i_diff(target_dir_i, state->chosen_directions_i[depth - 1], n);
    double cost = path_cost + smooth_cost;

    cost *= pow(state->discount_factor, depth - 1);

    double estimated_goal_result = goal_result_cost(vfh);

    return (float)(cost + estimated_goal_result);
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
        vfh->next_vfh_pluses = vfh_plus_next_from(s, vfh);
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

    // vfh_plus_t *vfh = *(vfh_plus_t**)new_state;
    // printf("New vfh state at depth %d, direction %d\n", vfh->depth, vfh->direction_i);

    return true;
}

void vfh_plus_destroy(void *state)
{
    vfh_plus_t *vfh = (vfh_plus_t*)state;
    zarray_destroy(vfh->next_vfh_pluses);
    free(vfh->binary_histogram);
    free(vfh->masked_histogram);
}

vfh_star_result_t *vfh_star_update(drive_to_wp_state_t *state,
                    double target_x, double target_y,
                    double min_turning_r, double vehicle_diam)
{
    state->target_x = target_x;
    state->target_y = target_y;
    state->min_turning_r = min_turning_r;
    state->vehicle_diam = vehicle_diam;

    double dx = target_x - state->xyt[0];
    double dy = target_y - state->xyt[1];
    double dist = sqrt(dx * dx + dy * dy);
    int needed_depth = (int)ceil(dist / state->step_distance);
    state->star_depth = min(needed_depth, state->goal_depth);

    if (!state->last_grid_map) {
        // *target_heading = state->chosen_direction; // use old value
        return NULL;
    }
    if (!state->vfh_has_inited) {
        delayed_initialize_vfh_star(state);
        state->vfh_has_inited = true;
    }

    vfh_plus_t *initial_vfh = calloc(1, sizeof(vfh_plus_t));
    initial_vfh->state = state;
    initial_vfh->direction_i = -1;
    initial_vfh->effective_dir = 0;
    initial_vfh->next_vfh_pluses = NULL;
    initial_vfh->masked_histogram = NULL;

    size_t binary_hist_size = state->polar_sections * sizeof(*initial_vfh->binary_histogram);
    initial_vfh->binary_histogram = malloc(binary_hist_size);

    if (!state->binary_histogram_prior) {
        state->binary_histogram_prior = malloc(binary_hist_size);
        memset(state->binary_histogram_prior, 0, binary_hist_size);
    }

    // memcpy(initial_vfh->binary_histogram, state->binary_histogram_prior, binary_hist_size);
    memset(initial_vfh->binary_histogram, 1, binary_hist_size);

    memcpy(initial_vfh->xyt, state->xyt, sizeof(initial_vfh->xyt));
    calculate_derived_vfh_properties(state, initial_vfh);

    // printf("\nSetting up new A* problem\n");

    // set up an A* problem
    general_search_problem_t p = { 0 };
    p.initial_state = initial_vfh;
    p.is_goal = is_goal;
    p.step_cost = step_cost;
    p.ordering_cost = ordering_cost;
    p.expand_state = expand_state;
    p.next_new_state = next_new_state;
    p.destroy_state = vfh_plus_destroy;
    p.allow_cycles = true; // duplicate states won't happen anyway
    // p.debugging = true;
    populate_with_priority_queue(&p);

    gen_search_node_t *result = tree_search(&p);
    if (result) {
        // we just need the top level choice of direction!
        gen_search_node_t *parent = result;
        while (parent->depth > 1) {
            parent = parent->parent;
        }

        vfh_plus_t *vfh = (vfh_plus_t*)parent->state;
        double section_i_to_rads = (2 * M_PI) / state->polar_sections;
        double chosen_direction = mod2pi(vfh->direction_i * section_i_to_rads);

        // copy to use as prior in next update
        memcpy(state->binary_histogram_prior, ((vfh_plus_t*)parent->parent->state)->binary_histogram,
               sizeof(*vfh->binary_histogram) * state->polar_sections);

        vfh_star_result_t *vfh_result = calloc(1, sizeof(vfh_star_result_t));
        vfh_result->p = p;
        vfh_result->node = result;
        vfh_result->target_heading = chosen_direction;
        vfh_result->cost = result->path_cost;

        vfh_result->target_headings_i =
            calloc(state->goal_depth, sizeof(*vfh_result->target_headings_i));
        gen_search_node_t *node = result;
        while (node->depth > 0) {
            vfh_plus_t *node_vfh = (vfh_plus_t*)node->state;
            vfh_result->target_headings_i[node->depth - 1] = node_vfh->direction_i;
            node = node->parent;
        }

        // render_vfh_star(state, result);
        // general_search_result_destroy(&p, result);
        return vfh_result;
    }
    // printf("Found no solution after expanding %d nodes.\n", p.expansion_count);
    // *target_heading = state->chosen_direction;
    return NULL;
}

void vfh_star_result_destroy(vfh_star_result_t *result)
{
    general_search_result_destroy(&result->p, result->node);
    free(result->p.initial_state);
    free(result);
}

void vfh_release_state(drive_to_wp_state_t *state)
{
    zarray_destroy(state->precomp_circle_lines);
    free(state->precomp_radians);
    free(state->precomp_invdist);
    // free(state->precomp_enlargements);
    free(state->precomp_magnitudes);
    free(state->binary_histogram_prior);
}
