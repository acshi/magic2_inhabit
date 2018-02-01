#pragma once

#include "drive_to_wp_state.h"

typedef struct vfh_plus {
    drive_to_wp_state_t *state;
    double xyt[3];
    int direction_i; // 0 is in the x-direction. counter-clock-wise is positive.
    double target_dir; // angle from our xyt to state target xy
    double effective_dir; // not the target direction as above, but that of actual expected movement
    zarray_t *next_vfh_pluses; // of vfh_plus_t
    uint8_t *masked_histogram;
    double star_active_d; // active diameter used for this vfh plus iteration
    double star_step_dist;
} vfh_plus_t;

void initialize_vfh_star(drive_to_wp_state_t *state, config_t *config);
double vfh_star_update(drive_to_wp_state_t *state, double target_x, double target_y, double min_turning_r);
