#pragma once

#include "drive_to_wp_state.h"

typedef struct vfh_plus {
    drive_to_wp_state_t *state;
    double xyt[3];
    int direction_i; // 0 is in the x-direction. counter-clock-wise is positive.
    zarray_t *next_vfh_pluses; // of vfh_plus_t
} vfh_plus_t;

void initialize_vfh_star(drive_to_wp_state_t *state, config_t *config);
void vfh_star_update(drive_to_wp_state_t *state);
