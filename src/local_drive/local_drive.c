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
#include "lcmtypes/drive_command_t.h"
#include "vx/vx.h"
#include "vx/webvx.h"


#define RENDER_HZ 10
#define CONTROL_HZ 10

// 19.5 in baseline == 49.5 cm
// 23.5 in width == 59.7 cm
// 29.0 in length == 73.7 cm

#define WIDTH 0.597
#define LENGTH 0.737
#define RADIUS (WIDTH/2.0)

// Robot potential field navigation.
//
// Inputs: velodyne data
// Ouput: diff drive
typedef struct state state_t;
struct state
{
    lcm_t *lcm;

    vx_world_t *vw;
    webvx_t *webvx;

    timeutil_rest_t *render_timer;
    timeutil_rest_t *timer;

    v2p_t *v2p;
    zarray_t *curr_pts;

    pose_t *last_pose;
    float last_gradient[2];
    float desired_direction[2];
    int stop;
};

static void on_drive_command(const lcm_recv_buf_t *rbuf,
                    const char *channel,
                    const drive_command_t *msg,
                    void *user) {
    state_t* state = user;
    state->stop = msg->stop;
    state->desired_direction[0] = 100*cos(msg->target_yaw);
    state->desired_direction[1] = 100*sin(msg->target_yaw);
    if (state->stop) {
        printf("Stop\n");
    } else {
        printf("yaw cmd: %f\n", msg->target_yaw*180.0/3.14159);
    }
}

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
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");
    state->render_timer = timeutil_rest_create();
    state->timer = timeutil_rest_create();

    config_t *config = NULL;
    if (getopt_was_specified(gopt, "config")) {
        config = config_create_path(getopt_get_string(gopt, "config"));
    }

    float S2B[16];
    config_get_S2B(config, S2B);
    state->v2p = v2p_create(S2B);

    state->curr_pts = zarray_create(sizeof(float[2]));

    state->desired_direction[0] = 100;
    state->desired_direction[1] = 0;
    state->stop = 1;

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

static void render_gradient(state_t* state) {
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "gradient");
    vx_buffer_add_back(vb,
            vxo_lines(vx_resource_make_attr_f32_copy((float[]) {0, 0, state->last_gradient[0], state->last_gradient[1]}, 4, 2),
                vx_red,
                1),
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
    timeutil_timer_stop(state->render_timer);
    if (timeutil_timer_timeout(state->render_timer, 1.0/RENDER_HZ)) {
        render_points(state);
        render_gradient(state);
        timeutil_timer_reset(state->render_timer);
    } else {
        timeutil_timer_start(state->render_timer);
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


static void on_create_canvas(vx_canvas_t *vc, const char *name, void *user)
{
    state_t *state = user;

    vx_canvas_set_title(vc, "local-drive");
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
}

static void on_destroy_canvas(vx_canvas_t *vc, void *user) {}

void command_robot(state_t* state) {
    if (state->stop) {
        return;
    }

    pose_t pose;
    int ret = v2p_get_pose(state->v2p, utime_now(), &pose);
    if (ret == -2) {
        printf("Outside interpolation range\n");
        return;
    }

    double curr_xyt_d[3];
    doubles_quat_xyz_to_xyt(pose.orientation, pose.pos, curr_xyt_d);
    float curr_xyt[3] = {curr_xyt_d[0], curr_xyt_d[1], curr_xyt_d[2]};

    float gradient[2] = {0};

    // Calculate potentials from lidar.
    for (int i = 0; i <  zarray_size(state->curr_pts); i++) {
        //gradient += k*(curr_xy - pt)/norm(curr_xy-pt)^3
        float pt[2];
        zarray_get(state->curr_pts, i, &pt);

        float direction[2];
        floats_subtract(curr_xyt, pt, 2, direction);

        float scale = 1/powf(floats_squared_magnitude(direction, 2), 1.5);

        floats_scale(scale, direction, 2, direction);
        floats_add(direction, gradient, 2, gradient);
    }
    floats_add(gradient, state->desired_direction, 2, gradient);

    printf("%f\n", floats_magnitude(gradient, 2));

    // TODO: Fix local minima issue.
    // Convert to DD command and publish.
    floats_normalize(gradient, 2, gradient);
    float ct = cos(curr_xyt[2]);
    float st = sin(curr_xyt[2]);
    float linear = 0.4*(ct*gradient[0] + st*gradient[1]);
    float rotational = 0.4*(-st*gradient[0] + ct*gradient[1]);
    diff_drive_t msg;
    msg.utime = utime_now();
    msg.left_enabled = 1;
    msg.right_enabled = 1;
    msg.left = linear - rotational;
    msg.right = linear + rotational;
    diff_drive_t_publish(state->lcm, "DIFF_DRIVE", &msg);

    state->last_gradient[0] = ct*gradient[0] + st*gradient[1];
    state->last_gradient[1] = -st*gradient[0] + ct*gradient[1];
}

int main(int argc, char **argv)
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, 'c', "config", "", "Robot config"); // XXX When in magic, use util
    getopt_add_int(gopt, 'p', "port", "8204", "vx port");
    getopt_add_string(gopt, '\0', "pose-channel", "POSE", "lcm pose channel");
    getopt_add_string(gopt, '\0', "velo-channel", "VELODYNE_DATA", "lcm velo channel");

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
    drive_command_t_subscribe(state->lcm, "DRIVE_COMMAND", on_drive_command, state);
    timeutil_timer_start(state->render_timer);
    timeutil_timer_start(state->timer);

    while (1) {
        timeutil_timer_stop(state->timer);
        if (timeutil_timer_timeout(state->timer, 1.0/CONTROL_HZ)) {
            timeutil_timer_reset(state->timer);
            command_robot(state);
        } else {
            timeutil_timer_start(state->timer);
            lcm_handle_timeout(state->lcm, 1000);
        }
    }
    return 0;
}
