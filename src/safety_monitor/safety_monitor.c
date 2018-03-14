#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include "common/config.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "common/math_util.h"
#include "common/time_util.h"
#include "common/velo_to_points.h"
#include "lcmtypes/diff_drive_t.h"
#include "vx/vx.h"
#include "vx/webvx.h"

#define RENDER_HZ 10

// 19.5 in baseline == 49.5 cm
// 23.5 in width == 59.7 cm
// 29.0 in length == 73.7 cm

#define WIDTH 0.597
#define LENGTH 0.737
#define RADIUS (WIDTH/2.0)



// Robot safety monitor.
//
// Inputs: diff drives, velodyne data
// Ouput: Scales down diff drive based on distance to closest obstacle
typedef struct state state_t;
struct state
{
    lcm_t *lcm;

    vx_world_t *vw;
    webvx_t *webvx;

    // Internal state
    timeutil_rest_t *timer;

    v2p_t *v2p;
    //sweep_t *last_sweep;
    zarray_t *curr_pts;
    zarray_t *coll_pts;

    pose_t *last_pose;

    double safety_radius;
    double scale_linear_slope;
    double scale_rotational_slope;
    double scale_linear_min;
    double scale_rotational_min;
};

static void config_get_S2B(config_t *config, float S2B[16])
{
    if (config == NULL) {
        floats_mat44_identity(S2B);
        fprintf(stderr, "WRN: Config contains no velodyne transform\n");
        return;
    }

    const zarray_t *xform_xyz = config_require_doubles(config, "velodyne.xyz_m");
    const zarray_t *xform_rpy = config_require_doubles(config, "velodyne.rpy_deg");

    float xyzrpy[6];
    for (int i = 0; i < 3; i++) {
        double v;
        zarray_get(xform_xyz, i, &v);
        xyzrpy[i] = (float)v;

        zarray_get(xform_rpy, i, &v);
        xyzrpy[i+3] = (float)(to_radians(v));
    }

    floats_xyzrpy_to_mat44(xyzrpy, S2B);
}

static state_t *state_create(getopt_t *gopt)
{
    state_t *state = calloc(1, sizeof(state_t));

    if (getopt_get_bool(gopt, "human-operator")) {
        state->safety_radius = 0.75;
        state->scale_linear_min = 0.2;
        state->scale_rotational_min = 0.3;
        state->scale_linear_slope = 1.0;
        state->scale_rotational_slope = 1.0;
    } else {
        state->safety_radius = 0.75;
        state->scale_linear_min = 0.0;
        state->scale_rotational_min = 0.0;
        state->scale_linear_slope = 1.0;
        state->scale_rotational_slope = 1.0;
    }

    state->lcm = lcm_create(NULL);

    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");

    state->timer = timeutil_rest_create();

    config_t *config = NULL;
    if (getopt_was_specified(gopt, "config")) {
        config = config_create_path(getopt_get_string(gopt, "config"));
    }

    float S2B[16];
    config_get_S2B(config, S2B);
    state->v2p = v2p_create(S2B);

    state->curr_pts = zarray_create(sizeof(float[2]));

    return state;
}

static void render_robot(state_t *state, float *color)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "robot");
    vx_buffer_add_back(vb, vxo_chain(vxo_matrix_scale(LENGTH),
                                     vxo_robot_solid(color),
                                     NULL),
                       NULL);

    vx_buffer_add_back(vb, vxo_chain(vxo_matrix_translate(0.0, 0.0, 0),
                                     vxo_matrix_scale(RADIUS),
                                     vxo_circle_line(vx_white, 1),
                                     NULL),
                       NULL);

    vx_buffer_add_back(vb, vxo_chain(vxo_matrix_scale3(LENGTH, WIDTH, 1),
                                     vxo_square_line(vx_cyan, 1),
                                     NULL),
                       NULL);
    vx_buffer_swap(vb);
}

static void render_points(state_t *state)
{
    zarray_t *pts = state->curr_pts;

    double xyt[3];
    doubles_quat_xyz_to_xyt(state->last_pose->orientation, state->last_pose->pos, xyt);
    double xyt_inv[3];
    doubles_xyt_inv(xyt, xyt_inv);
    double M[16];
    doubles_xyt_to_mat44(xyt_inv, M);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "points");
    vx_buffer_add_back(vb,
            vxo_chain(vxo_matrix(M),
                       vxo_points(vx_resource_make_attr_f32_copy((float*)pts->data,
                                                                 zarray_size(pts)*2,
                                                                 2),
                                  vx_yellow,
                                  1), NULL),
                       NULL);
    vx_buffer_swap(vb);
}

static void on_pose(const lcm_recv_buf_t *rbuf,
                    const char *channel,
                    const pose_t *msg,
                    void *user)
{
    state_t *state = user;

    v2p_add_pose(state->v2p, msg);

    if (state->last_pose) {
        pose_t_destroy(state->last_pose);
    }
    state->last_pose = pose_t_copy(msg);

    // Render viz updates periodically
    timeutil_timer_stop(state->timer);
    if (timeutil_timer_timeout(state->timer, 1.0/RENDER_HZ)) {
        render_points(state);
        timeutil_timer_reset(state->timer);
    } else {
        timeutil_timer_start(state->timer);
    }
}

static void on_velo(const lcm_recv_buf_t *rbuf,
                    const char *channel,
                    const raw_t *msg,
                    void *user)
{
    state_t *state = user;
    v2p_add_velo(state->v2p, msg);
    zarray_t* pts = v2p_get_sweep(state->v2p);
    if (pts) {
        zarray_destroy(state->curr_pts);
        state->curr_pts = pts;
    }
}

float clamp(float d, float min, float max) {
    float t = d < min ? min : d;
    return t > max ? max : t;
}


static void on_dd(const lcm_recv_buf_t *rbuf,
                  const char *channel,
                  const diff_drive_t *dd,
                  void *user)
{
    state_t *state = user;

    pose_t pose;
    int ret = v2p_get_pose(state->v2p, dd->utime, &pose);
    if (ret == -2) {
        printf("Outside interpolation range\n");
        return;
    }

    double curr_xyt_d[3];
    doubles_quat_xyz_to_xyt(pose.orientation, pose.pos, curr_xyt_d);
    float curr_xyt[3] = {curr_xyt_d[0], curr_xyt_d[1], curr_xyt_d[2]};

    float curr_xyt_inv[3];
    floats_xyt_inv(curr_xyt, curr_xyt_inv);

    float M[16];
    floats_xyt_to_mat44(curr_xyt_inv, M);

    // Calculating the 1/pseudo_distance because it is easier to calculate.
    // psuedo_distance is just a heuristic for how much a point should affect the robot's motion.
    float inv_pseudo_distance = 0.0;

    // Calculate min distance to robot center.
    for (int j = 0; j < zarray_size(state->curr_pts); j++) {
        float xyz[3];
        xyz[2] = 0;
        zarray_get(state->curr_pts, j, xyz);

        float dxyz[3];
        floats_mat44_transform_xyz(M, xyz, dxyz);

        // If the point is in the opposite of the commanded direction, ignore it.
        if ((dd->left + dd->right) * dxyz[0] < 0) {
          continue;
        }

        // Scale "distance" based on cos theta
        if (dxyz[0] < 0) {
          dxyz[0] = -dxyz[0];
        }
        inv_pseudo_distance = max(inv_pseudo_distance, dxyz[0]/ (dxyz[0]*dxyz[0] + dxyz[1]*dxyz[1]));
    }

    float pseudo_distance = 1000;
    if (inv_pseudo_distance > 0) {
      pseudo_distance = 1.0 / inv_pseudo_distance;
    }


    // Calculate scale factor for DD.
    float scale_linear = clamp(
            state->scale_linear_slope *
                (pseudo_distance - state->safety_radius) +
                state->scale_linear_min,
            state->scale_linear_min,
            1.0);
    float scale_rotational = clamp(
            state->scale_rotational_slope *
                (pseudo_distance - state->safety_radius) +
                state->scale_rotational_min,
            state->scale_rotational_min,
            1.0);

    // Clamp DD if scale < DD
    float dd_linear = (dd->right + dd->left) / 2.0;
    float dd_rotational = (dd->right - dd->left) / 2.0;

    if (scale_linear < fabs(dd_linear) || scale_rotational < fabs(dd_rotational)) {
        diff_drive_t dd_override;
        dd_override.utime = utime_now();
        dd_override.left_enabled = dd_override.right_enabled = 1;

        if (scale_linear < fabs(dd_linear)) {
            dd_linear = copysign(scale_linear, dd_linear);
        }
        if (scale_rotational < fabs(dd_rotational)) {
            dd_rotational = copysign(scale_rotational, dd_rotational);
        }
        dd_override.right = dd_linear + dd_rotational;
        dd_override.left = dd_linear - dd_rotational;
        diff_drive_t_publish(state->lcm, "DIFF_DRIVE_HUMAN_OVERRIDE", &dd_override);
    }
}

static void on_create_canvas(vx_canvas_t *vc, const char *name, void *user)
{
    state_t *state = user;

    vx_canvas_set_title(vc, "safety-monitor");
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
}

static void on_destroy_canvas(vx_canvas_t *vc, void *user) {}

int main(int argc, char **argv)
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, 'c', "config", "", "Robot config"); // XXX When in magic, use util
    getopt_add_int(gopt, 'p', "port", "8203", "vx port");
    getopt_add_string(gopt, '\0', "pose-channel", "POSE", "lcm pose channel");
    getopt_add_string(gopt, '\0', "velo-channel", "VELODYNE_DATA", "lcm velo channel");
    getopt_add_bool(gopt, '\0', "human-operator", 0, "Relax settings for a human operator.");

    if (!getopt_parse(gopt, argc, argv, 0)) {
        fprintf(stderr, "ERR: getopt_parse\n");
        exit(-1);
    }

    if (getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [opts] diff_drive_channel_1 ...\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    state_t *state = state_create(gopt);
    printf("INFO: webvx server on port %d\n", getopt_get_int(gopt, "port"));
    webvx_define_canvas(state->webvx,
                        "safety-monitor",
                        on_create_canvas,
                        on_destroy_canvas,
                        state);
    render_robot(state, vx_cyan);

    pose_t_subscribe(state->lcm, getopt_get_string(gopt, "pose-channel"), on_pose, state);
    raw_t_subscribe(state->lcm, getopt_get_string(gopt, "velo-channel"), on_velo, state);

    const zarray_t *extra_args = getopt_get_extra_args(gopt);
    if (zarray_size(extra_args) == 0) {
      fprintf(stderr, "ERR: no diff drive channels to monitor");
      exit(1);
    }
    for (int i = 0; i < zarray_size(extra_args); i++) {
        char *dd_chan = NULL;
        zarray_get(extra_args, i, &dd_chan);
        printf("INFO: monitoring diff_drive_t channel %s\n", dd_chan);
        diff_drive_t_subscribe(state->lcm, dd_chan, on_dd, state);
    }

    timeutil_timer_start(state->timer);
    while (1) {
        usleep(1000); // TODO Necessary?
        lcm_handle_timeout(state->lcm, 1000);
    }
    return 0;
}
