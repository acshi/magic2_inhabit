#pragma once

#include "drive_to_wp_state.h"

typedef struct vfh_plus {
    drive_to_wp_state_t *state;
    struct vfh_plus *parent;
    double xyt[3];
    int direction_i; // 0 is in the x-direction. counter-clock-wise is positive.
    double target_dir; // angle from our xyt to state target xy
    double effective_dir; // not the target direction as above, but that of actual expected movement
    zarray_t *next_vfh_pluses; // of vfh_plus_t
    uint8_t *binary_histogram;
    uint8_t *masked_histogram;
    double star_active_d; // active diameter used for this vfh plus iteration
    double star_step_dist;

    // these describe the section limits in the parent's masked histogram
    // that our direction_i was chosen from. involved in calculating costs.
    int section_left_dir_i;
    int section_right_dir_i;

    bool is_early_termination;

    double min_turning_r;
    int depth;
    float step_cost; // cache calculation value
} vfh_plus_t;

typedef struct vfh_star_result {
    general_search_problem_t p;
    gen_search_node_t *node;
    int *target_headings_i;
    double target_heading;
    double cost;
} vfh_star_result_t;

void initialize_vfh_star(drive_to_wp_state_t *state, config_t *config);
vfh_star_result_t *vfh_star_update(drive_to_wp_state_t *state,
                    double target_x, double target_y,
                    double min_turning_r, double vehicle_diam);
void vfh_star_result_destroy(vfh_star_result_t *result);
void vfh_release_state(drive_to_wp_state_t *state);

void vfh_plus_update_histograms(drive_to_wp_state_t *state, vfh_plus_t *vfh);
