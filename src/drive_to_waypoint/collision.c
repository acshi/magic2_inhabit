#include "collision.h"

void rasterize_poly_line(int *buff_x0, int *buff_x1, int startX, int startY, int endX, int endY)
{
    // Bresenham's Line Drawing, as applied to rasterizing a convex polygon
    int cx = startX;
    int cy = startY;

    int dx = abs(endX - startX);
    int dy = abs(endY - startY);

    int sx = startX < endX ? 1 : -1;
    int sy = startY < endY ? 1 : -1;

    int err = dx - dy;

    while(1) {
        if (sy > 0) {
            buff_x0[cy] = cx;
        } else if (sy < 0) {
            buff_x1[cy] = cx;
        } else {
            if (buff_x0[cy] == -1) {
                buff_x0[cy] = cx;
            } else {
                buff_x0[cy] = min(buff_x0[cy], cx);
            }
            if (buff_x1[cy] == -1) {
                buff_x1[cy] = cx;
            } else {
                buff_x1[cy] = max(buff_x1[cy], cx);
            }
        }

        if ((cx == endX) && (cy == endY)) {
            return;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            cx = cx + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            cy = cy + sy;
        }
    }
}

bool obstacle_in_region(grid_map_t *gm, double *xyt,
                        double left_dist, double right_dist,
                        double forward_dist, double backward_dist,
                        double *minimum_hit_margin)
{
    *minimum_hit_margin = 0;

    if (!gm) {
        return true;
    }

    double cos_theta = cos(xyt[2]);
    double sin_theta = sin(xyt[2]);

    double xs[4];
    double ys[4];

    xs[0] = xyt[0] - left_dist * sin_theta - backward_dist * cos_theta;
    ys[0] = xyt[1] + right_dist * cos_theta - backward_dist * sin_theta;

    xs[1] = xyt[0] - left_dist * sin_theta + forward_dist * cos_theta;
    ys[1] = xyt[1] + right_dist * cos_theta + forward_dist * sin_theta;

    xs[2] = xyt[0] + right_dist * sin_theta + forward_dist * cos_theta;
    ys[2] = xyt[1] - left_dist * cos_theta + forward_dist * sin_theta;

    xs[3] = xyt[0] + right_dist * sin_theta - backward_dist * cos_theta;
    ys[3] = xyt[1] - left_dist * cos_theta - backward_dist * sin_theta;

    int ixs[4];
    int iys[4];
    int minIY = -1;
    int maxIY = -1;
    for (int i = 0; i < 4; i++) {
        ixs[i] = gridmap_get_index_x(gm, xs[i]);
        iys[i] = gridmap_get_index_y(gm, ys[i]);
        if (minIY == -1 || iys[i] < minIY) {
            minIY = iys[i];
        }
        if (maxIY == -1 || iys[i] > maxIY) {
            maxIY = iys[i];
        }
        // printf("(%d, %d), ", ixs[i], iys[i]);
    }
    // printf("\n");

    // grid map location index of the robot origin
    int r_ix = gridmap_get_index_x(gm, xyt[0]);
    int r_iy = gridmap_get_index_y(gm, xyt[1]);

    // rectangle rasterization: https://stackoverflow.com/questions/10061146/how-to-rasterize-rotated-rectangle-in-2d-by-setpixel
    int buff_x0[gm->height];
    int buff_x1[gm->height];
    for (int i = 0; i < gm->height; i++) {
        buff_x0[i] = -1;
        buff_x1[i] = -1;
    }

    rasterize_poly_line(buff_x0, buff_x1, ixs[0], iys[0], ixs[1], iys[1]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[1], iys[1], ixs[2], iys[2]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[2], iys[2], ixs[3], iys[3]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[3], iys[3], ixs[0], iys[0]);

    double min_dist_sq = DBL_MAX;

    for (int y = minIY; y <= maxIY; y++) {
        if (buff_x0[y] > buff_x1[y]) {
            printf("Wrong ordering at y %d\n", y);
        }
        for (int x = buff_x0[y]; x <= buff_x1[y]; x++) {
            if (!(gm->data[y * gm->width + x] & GRID_FLAG_TRAVERSABLE)) {
                // printf("Collision at %d, %d\n", x, y);
                double dist_sq = isq(x - r_ix) + isq(y - r_iy);
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                }
            }
        }
    }

    if (min_dist_sq == DBL_MAX) {
        return false;
    }

    *minimum_hit_margin = sqrt(min_dist_sq) * gm->meters_per_pixel;
    return true;
}

bool obstacle_ahead(drive_to_wp_state_t *state, double *slow_down_factor)
{
    double left_dist = state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_speed = max(0, state->forward_vel);
    double forward_dist = max(state->min_forward_distance, state->min_forward_per_mps * forward_speed);
    double backward_dist = 0;
    double margin;
    bool has_obstacle = obstacle_in_region(state->last_grid_map, state->xyt, left_dist, right_dist, forward_dist, backward_dist, &margin);
    if (has_obstacle) {
        double max_dist = sqrt(sq(left_dist) + sq(forward_dist));
        double min_dist = min(left_dist, forward_dist);
        if (margin < min_dist) {
            *slow_down_factor = 0;
        } else {
            *slow_down_factor = (margin - min_dist) / (max_dist - min_dist);
        }
        // printf("Ahead margin found: %7.3f w/ slowdown: %7.3f ", margin, *slow_down_factor);
    } else {
        *slow_down_factor = 1;
    }
    return *slow_down_factor == 0;
}

bool obstacle_behind(drive_to_wp_state_t *state, double *slow_down_factor)
{
    double left_dist = state->min_side_back_distance + state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_dist = 0;
    double backward_speed = max(0, -state->forward_vel);
    double backward_dist = max(state->min_forward_distance, state->min_forward_per_mps * backward_speed);
    double margin;
    bool has_obstacle = obstacle_in_region(state->last_grid_map, state->xyt, left_dist, right_dist, forward_dist, backward_dist, &margin);
    if (has_obstacle) {
        double max_dist = sqrt(sq(left_dist) + sq(backward_dist));
        double min_dist = min(left_dist, backward_dist);
        if (margin < min_dist) {
            *slow_down_factor = 0;
        } else {
            *slow_down_factor = (margin - min_dist) / (max_dist - min_dist);
        }
        // printf("Behind margin found: %7.3f w/ slowdown: %7.3f ", margin, *slow_down_factor);
    } else {
        *slow_down_factor = 1;
    }
    return *slow_down_factor == 0;
}

bool obstacle_by_sides(drive_to_wp_state_t *state, double *slow_down_factor)
{
    double left_dist = state->min_side_turn_distance + state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_dist = state->min_forward_distance;
    double backward_dist = state->min_forward_distance;
    double margin;
    bool has_obstacle = obstacle_in_region(state->last_grid_map, state->xyt, left_dist, right_dist, forward_dist, backward_dist, &margin);
    if (has_obstacle) {
        double max_dist = sqrt(sq(left_dist) + sq(forward_dist));
        double min_dist = min(left_dist, forward_dist);
        if (margin < min_dist) {
            *slow_down_factor = 0;
        } else {
            *slow_down_factor = (margin - min_dist) / (max_dist - min_dist);
        }
        // printf("Sides margin found: %7.3f w/ slowdown: %7.3f \n", margin, *slow_down_factor);
    } else {
        *slow_down_factor = 1;
    }
    return *slow_down_factor == 0;
}
