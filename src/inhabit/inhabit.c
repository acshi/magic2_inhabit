#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <sys/select.h>

#include "common/doubles.h"
#include "common/time_util.h"
#include "common/magic_util.h"
#include "lcm/lcm.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/robot_command_t.h"
#include "lcmtypes/robot_task_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/lcmdoubles_t.h"
#include "lcm_handle_async.h"
#include "gui/taskgui/command_center.h"

// in meters
#define DEAD_BAND 0.05

typedef struct {
    lcm_t *lcm;
    pose_t *last_pose;
    lcmdoubles_t *last_l2g;

    double zero_xyt[3];
    double robot_xyt[3];

    int state_num;
    int robot_id;
} inhabit_t;

#define CONTROL_UPDATE_MS 100

bool has_reached_pos(inhabit_t *state, double *pos) {
    if (!state->last_pose) {// || !state->last_l2g) {
        return false;
    }

    // double local_pose[3] = {state->last_pose->pos[0], state->last_pose->pos[1], 0.0};
    // double global_pose[3];
    // doubles_xyt_mul(state->last_l2g->data, local_pose, global_pose);

    double diff_x = abs(pos[0] - state->robot_xyt[0]);
    double diff_y = abs(pos[1] - state->robot_xyt[1]);
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    return dist_sq <= DEAD_BAND * DEAD_BAND;
}

bool drive_to(inhabit_t *state, double* target_pos) {
    if (has_reached_pos(state, target_pos)) {
        return true;
    }

    int nwaypts = 1;

    robot_command_t *cmd = calloc(1, sizeof(robot_command_t));
    cmd->robotid = state->robot_id; // number of our robot
    cmd->task.task = ROBOT_TASK_T_GOTO_GLOBAL; // currently the only operational command
    cmd->ndparams = 4*nwaypts;
    cmd->dparams = malloc(4*nwaypts*sizeof(double));
    cmd->niparams = nwaypts;
    cmd->iparams = malloc(nwaypts*sizeof(int));

    for(int i = 0; i < nwaypts; i++) {
        cmd->dparams[4*i + 0] = target_pos[0]; // global target x position
        cmd->dparams[4*i + 0] = target_pos[1]; // y position
        cmd->dparams[4*i + 2] = NAN; // angle theta, NAN to say "don't care"
        cmd->dparams[4*i + 3] = DEAD_BAND; // achievement distance
        // (at which the way point is declared finished)

        cmd->iparams[i] = 0; // not currently used, but must be 0
    }
    robot_command_t_publish(state->lcm, "CMDS", cmd);
    robot_command_t_destroy(cmd);

    return false;
}

void drive_idle(inhabit_t *state) {
    robot_command_t *cmd = calloc(1, sizeof(robot_command_t));
    cmd->task.task = ROBOT_TASK_T_FREEZE;
    cmd->robotid = state->robot_id; // number of our robot
    robot_command_t_publish(state->lcm, "CMDS", cmd);
    robot_command_t_destroy(cmd);
}

void receive_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_t *msg, void *user)
{
    inhabit_t *state = user;
    if (state->last_pose && msg->utime <= state->last_pose->utime) {
        return;
    }
    if (state->last_pose) {
        pose_t_destroy(state->last_pose);
    }
    state->last_pose = pose_t_copy(msg);

    double received_xyt[3];
    doubles_quat_xyz_to_xyt(state->last_pose->orientation, state->last_pose->pos, received_xyt);

    state->robot_xyt[0] = received_xyt[0] - state->zero_xyt[0];
    state->robot_xyt[1] = received_xyt[1] - state->zero_xyt[1];
    state->robot_xyt[2] = mod2pi(received_xyt[2] - state->zero_xyt[2]);
}

void receive_l2g(const lcm_recv_buf_t *rbuf, const char *channel,
                  const lcmdoubles_t *msg, void *user)
{
    inhabit_t *state = user;
    if (state->last_l2g && msg->utime <= state->last_l2g->utime) {
        return;
    }
    if (state->last_l2g) {
        lcmdoubles_t_destroy(state->last_l2g);
    }
    state->last_l2g = lcmdoubles_t_copy(msg);
}

void drive_in_square(inhabit_t *state) {
    double way_points[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};

    if (drive_to(state, way_points[state->state_num])) {
        state->state_num++;
        if (state->state_num >= 4) {
            state->state_num = 0;
        }
    }
}

void reset_odometry(inhabit_t *state) {
    doubles_quat_xyz_to_xyt(state->last_pose->orientation, state->last_pose->pos, state->zero_xyt);
    state->robot_xyt[0] = 0;
    state->robot_xyt[1] = 0;
    state->robot_xyt[2] = 0;
}

// http://cc.byexamples.com/2007/04/08/non-blocking-user-input-in-loop-without-ncurses/
// is key board (stdin) data available?
bool kbhit()  {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);

    inhabit_t *state = calloc(1, sizeof(inhabit_t));
    state->robot_id = magic_util_id();
    if (state->robot_id == -1) {
        fprintf(stderr, "Could not determine robot's ID number. Is ROBOT_ID not set?\n");
        return 1;
    }

    state->lcm = lcm_create(NULL);
    if (!state->lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    pose_t_subscribe(state->lcm, "POSE", receive_pose, state);
    lcmdoubles_t_subscribe(state->lcm, "L2G", receive_l2g, state);

    printf("Inhabit Running!\n");

    bool started = false;
    printf("Hit Enter to start\n");
    printf("Attempting to drive in a counter-clockwise square...\n");

    while (1) {
        lcm_handle_async(state->lcm);
        if (!started && kbhit()) {
            started = true;
            reset_odometry(state);
        }
        if (started) {
            drive_in_square(state);
        }
        nanosleep(&(struct timespec){0, (CONTROL_UPDATE_MS * 1e6)}, NULL);
    }

    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
