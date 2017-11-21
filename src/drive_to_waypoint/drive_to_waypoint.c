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
#include "lcmtypes/waypoint_cmd_t.h"
#include "lcmtypes/lcmdoubles_t.h"
#include "acshi_common/lcm_handle_async.h"

#define HEADING_THRESH (4*M_PI/180.0)

typedef struct {
    lcm_t *lcm;
    pose_t *last_pose;
    robot_map_data_t *last_map_data;
    waypoint_cmd_t *last_cmd;

    double xyt[3];
    float achievement_dist;
} drive_to_wp_state_t;

#define CONTROL_UPDATE_MS 15

double dist_to_dest(drive_to_wp_state_t *state) {
    double diff_x = state->last_cmd->xyt[0] - state->xyt[0];
    double diff_y = state->last_cmd->xyt[1] - state->xyt[1];
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    return sqrt(dist_sq);
}

bool has_reached_dest(drive_to_wp_state_t *state) {
    if (!state->last_pose) {
        return false;
    }

    double diff_x = state->last_cmd->xyt[0] - state->xyt[0];
    double diff_y = state->last_cmd->xyt[1] - state->xyt[1];
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    return dist_sq <= state->last_cmd->achievement_dist * state->last_cmd->achievement_dist;
}

void receive_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_t *msg, void *user)
{
    drive_to_wp_state_t *state = user;
    if (state->last_pose && msg->utime <= state->last_pose->utime) {
        return;
    }
    if (state->last_pose) {
        pose_t_destroy(state->last_pose);
    }
    state->last_pose = pose_t_copy(msg);

    doubles_quat_xyz_to_xyt(state->last_pose->orientation, state->last_pose->pos, state->xyt);
}

void receive_robot_map_data(const lcm_recv_buf_t *rbuf, const char *channel,
                            const robot_map_data_t *msg, void *user)
{
    drive_to_wp_state_t *state = user;
    if (state->last_map_data && msg->utime <= state->last_map_data->utime) {
        return;
    }
    if (state->last_map_data) {
        robot_map_data_t_destroy(state->last_map_data);
    }
    state->last_map_data = robot_map_data_t_copy(msg);
}

void receive_waypoint_cmd(const lcm_recv_buf_t *rbuf, const char *channel,
                          const waypoint_cmd_t *msg, void *user)
{
    drive_to_wp_state_t *state = user;
    if (state->last_cmd && msg->utime <= state->last_cmd->utime) {
        return;
    }
    if (state->last_cmd) {
        waypoint_cmd_t_destroy(state->last_cmd);
    }
    state->last_cmd = waypoint_cmd_t_copy(msg);
}

double constrain(double val, double min_val, double max_val) {
    if (val < min_val) {
        return min_val;
    } else if (val > max_val) {
        return max_val;
    }
    return val;
}

void update_control(drive_to_wp_state_t *state)
{
    if (!state->last_cmd) {
        return;
    }

    // we want to drive in roughly a straight line while avoiding obstacles.
    // like with A*?
    // maybe just start with going in a straight line and not hitting things.
    double left_motor = 0;
    double right_motor = 0;

    waypoint_cmd_t *cmd = state->last_cmd;
    double target_heading = atan2(cmd->xyt[1] - state->xyt[1], cmd->xyt[0] - state->xyt[0]);
    double heading_err = mod2pi(target_heading - state->xyt[2]);
    if (fabs(heading_err) > HEADING_THRESH) {
        double mag = constrain(0.9 * heading_err, 0.25, 0.4);
        left_motor = -copysign(mag, heading_err);
        right_motor = -left_motor;
    } else {
        double dist = dist_to_dest(state);
        if (dist > cmd->achievement_dist) {
            double mag = constrain(0.5 * dist, 0.2, 0.6);
            left_motor = mag;
            right_motor = mag;
        }
    }

    diff_drive_t motor_cmd = {
        .utime = utime_now(),
        .left = left_motor,
        .left_enabled = true,
        .right = right_motor,
        .right_enabled = true
    };
    diff_drive_t_publish(state->lcm, "DIFF_DRIVE", &motor_cmd);

    //printf("Heading: %.3f Target Heading: %.3f dx: %.3f dy: %.3f L: %.3f R: %.3f\n", state->xyt[2], target_heading, cmd->xyt[0] - state->xyt[0], cmd->xyt[1] - state->xyt[1], left_motor, right_motor);
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);

    drive_to_wp_state_t *state = calloc(1, sizeof(drive_to_wp_state_t));

    state->lcm = lcm_create(NULL);
    if (!state->lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    pose_t_subscribe(state->lcm, "POSE", receive_pose, state);
    robot_map_data_t_subscribe(state->lcm, "ROBOT_MAP_DATA", receive_robot_map_data, state);
    waypoint_cmd_t_subscribe(state->lcm, "WAYPOINT_CMD", receive_waypoint_cmd, state);

    while (1) {
        lcm_handle_async(state->lcm);

        update_control(state);

        nanosleep(&(struct timespec){0, (CONTROL_UPDATE_MS * 1e6)}, NULL);
    }

    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
