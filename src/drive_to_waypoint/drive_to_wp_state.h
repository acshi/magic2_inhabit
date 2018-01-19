#pragma once

#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <sys/select.h>

#include "common/doubles.h"
#include "common/time_util.h"
#include "common/math_util.h"
#include "lcm/lcm.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/robot_map_data_t.h"
#include "lcmtypes/grid_map_t.h"
#include "lcmtypes/waypoint_cmd_t.h"
#include "lcmtypes/lcmdoubles_t.h"
#include "acshi_common/lcm_handle_async.h"

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
    double forward_vel;

    double xyt[3];

    bool stopped_for_obstacle;

    double vehicle_width;
    double min_side_distance;
    double min_forward_distance;
    double min_forward_per_mps;

    // GUI
    vx_world_t *vw;
    webvx_t *webvx;
} drive_to_wp_state_t;
