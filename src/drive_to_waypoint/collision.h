#pragma once

#include "drive_to_wp_state.h"

void rasterize_poly_line(int *buff_x0, int *buff_x1, int startX, int startY, int endX, int endY);
bool obstacle_in_region(grid_map_t *gm, double *xyt,
                        double left_dist, double right_dist,
                        double forward_dist, double backward_dist);
bool obstacle_ahead(drive_to_wp_state_t *state);
bool obstacle_behind(drive_to_wp_state_t *state);
bool obstacle_by_sides(drive_to_wp_state_t *state);
