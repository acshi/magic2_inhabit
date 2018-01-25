#include "vfh_star.h"

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

void vfh_read_config(drive_to_wp_state_t *state, config_t *config)
{
    state->polar_sections = config_require_int(config, "vfh_star.polar_sections");
    require_value_nonnegative(state->polar_sections, "vfh_star.polar_sections");

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
}

void initialize_vfh_star(drive_to_wp_state_t *state, config_t *config)
{
    vfh_read_config(state, config);

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

void update_polar_density(drive_to_wp_state_t *state)
{
    grid_map_t *gm = state->last_grid_map;

    int cx = gridmap_get_index_x(gm, state->xyt[0]);
    int cy = gridmap_get_index_y(gm, state->xyt[1]);

    for (int i = 0; i < state->polar_sections; i++) {
        state->polar_density[i] = 0;
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

            int rad_min_i = (int)((rads - enlargement_rad) * rads_to_section_i);
            int rad_max_i = (int)((rads + enlargement_rad) * rads_to_section_i);
            for (int j = rad_min_i; j <= rad_max_i; j++) {
                state->polar_density[j] += magnitude;
            }
        }
    }
}

void update_binary_polar_histogram(drive_to_wp_state_t *state)
{
    for (int i = 0; i < state->polar_sections; i++) {
        double d = state->polar_density[i];
        if (d < state->polar_density_traversable) {
            state->binary_polar_histogram[i] = 0;
        } else if (d > state->polar_density_occupied) {
            state->binary_polar_histogram[i] = 1;
        }
    }
}

void update_masked_polar_histogram(drive_to_wp_state_t *state)
{
    grid_map_t *gm = state->last_grid_map;

    int cx = gridmap_get_index_x(gm, state->xyt[0]);
    int cy = gridmap_get_index_y(gm, state->xyt[1]);

    double forward_rads = state->xyt[2];
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
            if (delta_rads > 0 && rads < left_rad_limit) {
                int left_dist_sq = isq(left_center_x - x) + isq(left_center_y - y);
                if (left_dist_sq < dist_sq_limit) {
                    left_rad_limit = rads;
                }
            }
            if (delta_rads <= 0 && rads > right_rad_limit) {
                int right_dist_sq = isq(right_center_x - x) + isq(right_center_y - y);
                if (right_dist_sq < dist_sq_limit) {
                    right_rad_limit = rads;
                }
            }

        }
    }
}
