#pragma once

#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <sys/select.h>

#include "common/gridmap.h"
#include "common/config.h"
#include "common/doubles.h"
#include "common/time_util.h"
#include "common/math_util.h"
#include "common/zarray.h"
#include "common/general_search.h"
#include "common/pid_ctrl.h"

#include "lcm/lcm.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/robot_map_data_t.h"
#include "lcmtypes/grid_map_t.h"
#include "lcmtypes/waypoint_cmd_t.h"
#include "lcmtypes/lcmdoubles_t.h"

#include "common/lcm_handle_async.h"
#include "velodyne_to_map2/gridmap2.h"

#include "vx/vx.h"
#include "vx/webvx.h"
#include "vx/vx_colors.h"
#include "vx/vxo_robot.h"
#include "vx/vxo_square.h"
#include "vx/vxo_circle.h"
#include "vx/vxo_image.h"

#define HEADING_THRESH (4*M_PI/180.0)
#define CONTROL_UPDATE_MS 15

typedef struct {
    lcm_t *lcm;
    pose_t *last_pose;
    robot_map_data_t *last_map_data;
    grid_map_t *last_grid_map;
    waypoint_cmd_t *last_cmd;

    double control_update_hz;
    bool debugging;

    double forward_vel;
    double xyt[3];

    // last commanded
    uint64_t last_motor_utime;
    double last_forward;
    double last_turning;

    double motor_low_pass_f;

    // for collision avoidance
    bool stopped_for_obstacle;
    bool has_obstacle_ahead;
    bool has_obstacle_behind;
    bool has_obstacle_by_sides;
    double vehicle_width;
    double min_side_distance;
    double min_forward_distance;
    double min_forward_per_mps;

    // for VFH*
    // Settings (from config)
    int polar_sections;
    int wide_opening_size; // in polar section divisions
    double active_diameter;
    double max_magnitude;
    double planning_clearance;
    double polar_density_traversable;
    double polar_density_occupied;
    double step_distance; // each stage of vfh* looks forward this far
    int goal_depth; // number of steps forward to look with vfh*. 1 makes it vfh+
    double discount_factor; // discounts costs by depth of node in search

    int cost_goal_oriented; // mu_1 in the paper
    int cost_smooth_path; // mu_2 in the paper
    int cost_smooth_commands; // mu_3 in the paper
    int cost_proj_goal_oriented; // mu_1' in the paper
    int cost_proj_smooth_path; // mu_2' in the paper
    int cost_proj_smooth_commands; // mu_3' in the paper

    // VFH* internal state
    bool vfh_has_inited;
    int star_depth;
    double target_x;
    double target_y;
    double min_turning_r;
    int chosen_direction_i;
    double chosen_direction;
    uint8_t *binary_histogram_prior;

    zarray_t *precomp_circle_lines;
    double *precomp_radians;
    double *precomp_enlargements;
    double *precomp_magnitudes;

    // State for lower-level PID control
    double max_velocity;
    pid_ctrl_t *velocity_pid;
    pid_ctrl_t *heading_pid;

    // GUI
    vx_world_t *vw;
    webvx_t *webvx;
} drive_to_wp_state_t;

static inline void require_value_nonnegative(double value, const char *var_name)
{
    if (value <= 0.0) {
        fprintf(stderr, "ERR: %s must be > 0. Currently: %f\n", var_name, value);
        exit(EXIT_FAILURE);
    }
}
