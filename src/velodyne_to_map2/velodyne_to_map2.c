// Removed smearing to reduce CPU usage

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include <lcm/lcm.h>

#include "gridmap2.h"

#include "lcmtypes/laser_t.h"
#include "lcmtypes/raw_t.h"
#include "lcmtypes/gps_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/robot_map_data_t.h"
#include "lcmtypes/grid_map_t.h"
#include "lcmtypes/mesh_t.h"

#include "common/config.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "common/http_advertiser.h"
#include "common/image_u8.h"
#include "common/getopt.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "common/interpolator.h"
#include "common/math_util.h"
#include "common/time_util.h"
#include "common/zarray.h"
#include "common/zqueue.h"

#include "common/magic_util.h"

#include "velodyne/april_velodyne.h"

#include "vx/vx.h"
#include "vx/webvx.h"

// 85 degrees would be a bit  more conservative and seems to work OK XXX
#define SLAM_SLOPE to_radians(80)
#define MAX_SLOPE to_radians(20)
#define MIN_RELATIVE_HEIGHT -0.5
#define MAX_RELATIVE_OBSTACLE_HEIGHT 1.5
#define MAX_FILL_IN_DIST 0.25

#define RAYCASTING_RANGE 5.0
#define RAYCASTING_STEP 0.01

#define GRID_MAP_SIZE 768

typedef struct point_accumulator {
    int64_t acc;
    float z_min;
    float z_max;
} point_accumulator_t;

typedef struct state {
    lcm_t* lcm;
    int64_t last_publish_utime;
    int64_t mesh_publish_utime;
    int64_t publish_delay_utime;
    int64_t mesh_delay_utime;
    char *map_channel;
    char *tx_channel;

    config_t *config;
    int robot_id;

    // Map storage
    bool map_init;
    double mpp;
    int line_nsteps;
    grid_map_t *gm;
    robot_map_data_t rmd_out;

    double xyt_local[3];
    raw_t *msg;

    int nrays;
    float *raycasting_offset_xy;

    // Terrain classification
    zarray_t *last_slice;

    // Pose
    interpolator_t *pose_interp;
    interpolator_t *gps_interp;

    // viewer fields
    bool debug;
    vx_world_t* vw;
    webvx_t* webvx;
    zarray_t *pts;
    zarray_t *intensities;

    // Velodyne management
    april_velodyne_t *velo;
    zqueue_t* velodyne_msgs;
    pthread_mutex_t velodyne_msgs_mutex;
} state_t;

void on_create_canvas(vx_canvas_t* vc, const char* name, void* impl) {
   state_t* state = (state_t*)impl;

   printf("[INFO] on create canvas\n");
   vx_layer_t* vl = vx_canvas_get_layer(vc, "default");

   vx_layer_set_world(vl, state->vw);
}

void on_destroy_canvas(vx_canvas_t* vc, void* impl) {
   printf("[INFO] on destroy canvas\n");
}

void signal_handler(int sig)
{
    exit(sig);
}

static void pose_handler(const lcm_recv_buf_t *rbuf,
                         const char *channel,
                         const pose_t *msg,
                         void *user)
{
    state_t *state = user;

    pthread_mutex_lock(&state->velodyne_msgs_mutex);
    interpolator_add(state->pose_interp, msg);
    pthread_mutex_unlock(&state->velodyne_msgs_mutex);
}

static void velodyne_callback(const lcm_recv_buf_t* rbuf,
                              const char* channel,
                              const raw_t* msg,
                              void* user)
{
    state_t* state = (state_t*)user;
    raw_t* msg_copy = raw_t_copy(msg);;

    pthread_mutex_lock(&state->velodyne_msgs_mutex);
    zqueue_push(state->velodyne_msgs, &msg_copy);
    while (zqueue_size(state->velodyne_msgs) > 90) {
        raw_t *vmsg;
        zqueue_pop(state->velodyne_msgs, &vmsg);
        raw_t_destroy(vmsg);
    }
    pthread_mutex_unlock(&state->velodyne_msgs_mutex);
}

void on_gps(const lcm_recv_buf_t *rbuf,
             const char *channel, const gps_t *msg, void *userdata)
{
    state_t *state = userdata;

    interpolator_add(state->gps_interp, msg);
}

int get_num_bit_set(unsigned long long data, int start_index) {
    data = data & (~0 << start_index);
    int num_bit_set = 0;
    while(data) {
        data &= (data - 1);
        num_bit_set++;
    }
    return num_bit_set;
}

static void render_data(state_t *state)
{
    if (zarray_size(state->pts) <= 0)
        return;
    zarray_t *pts = state->pts;
    zarray_t *intensities = state->intensities;

    double inv_xyt[3];
    doubles_xyt_inv(state->xyt_local, inv_xyt);
    if (state->debug) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "points");
        vx_resource_t *vr = vx_resource_make_attr_f32_copy((float*)pts->data,
                                                           3*zarray_size(pts),
                                                           3);
        vx_resource_t *vi = vx_resource_make_attr_f32_copy((float*)intensities->data,
                                                           zarray_size(intensities),
                                                           1);
        vx_buffer_add_back(vb,
                           vxo_matrix_xyt(inv_xyt),
                           //vxo_points(vr, vx_yellow, 2),
                           vxo_points_pretty(vr, vi, 3),
                           NULL);
        vx_buffer_swap(vb);
    }

    if (state->debug) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "gridmap");

        int w = state->gm->width;
        int h = state->gm->height;

        image_u8_t *im = image_u8_create(w, h);
        int is = im->stride;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                uint8_t v = state->gm->data[y*w + x];
                uint8_t gray = 0;
                if ((v & GRID_FLAG_TRAVERSABLE) && (v & GRID_FLAG_SLAMMABLE)) {
                    gray = (uint8_t)0xff;
                } else if (v & GRID_FLAG_TRAVERSABLE) {
                    gray = (uint8_t)0xdd;
                } else if (v & GRID_FLAG_SLAMMABLE) {
                    gray = (uint8_t)0;
                } else if (v & GRID_FLAG_OBSTACLE) {
                    gray = (uint8_t)0x77;
                } else {
                    gray = (uint8_t)0xaa;
                }
                im->buf[y*is + x] = gray;
            }
        }

        vx_buffer_add_back(vb,
                           vxo_chain(
                                     vxo_matrix_xyt(inv_xyt),
                                     vxo_matrix_translate(state->gm->x0,
                                                          state->gm->y0,
                                                          0),
                                     vxo_matrix_scale(state->gm->meters_per_pixel),
                                     vxo_image_u8(im, 0),
                                     NULL),
                           NULL);
        vx_buffer_swap(vb);

        image_u8_destroy(im);
    }

    zarray_clear(pts);
    zarray_clear(intensities);
}

static void draw_line(grid_map_t *gm, int xa, int ya, int xb, int yb, int nsteps)
{
    //double dist = sqrt(sq(xb-xa) + sq(yb-ya));
    //int nsteps = (int) (dist / gm->meters_per_pixel + 1);

    for (int i = 0; i < nsteps; i++) {
        double alpha = ((double) i)/nsteps;
        double x = xa*alpha + xb*(1-alpha);
        double y = ya*alpha + yb*(1-alpha);

        int ix = round(x);
        int iy = round(y);

        if (ix < 0 || ix >= gm->width || iy < 0 || iy >= gm->width)
            break;
        int idx = iy*gm->width + ix;
        if (gm->data[idx] & GRID_FLAG_OBSTACLE)
            break;
        gm->data[idx] |= GRID_FLAG_TRAVERSABLE;
    }
}

static void line_of_sight(state_t *state)
{
    // Assumes the robot was stationary when computing observed free space.
    // This is close enough to reality at the speeds we travel.
    double *rxy = state->rmd_out.xyt_local;
    int ix = gridmap_get_index_x(state->gm, rxy[0]);
    int iy = gridmap_get_index_y(state->gm, rxy[1]);

    // Clean up around robot.
    for (int y = iy - 3; y < iy + 3; y++) {
        for (int x = ix - 3; x < ix + 3; x++) {
            gridmap_set_value_index(state->gm, x, y, GRID_FLAG_TRAVERSABLE);
        }
    }

    for (int i = 0; i < state->nrays; i++) {
        int x = gridmap_get_index_x(state->gm, rxy[0] + state->raycasting_offset_xy[2*i + 0]);
        int y = gridmap_get_index_y(state->gm, rxy[1] + state->raycasting_offset_xy[2*i + 1]);
        draw_line(state->gm, x, y, ix, iy, state->line_nsteps);
    }
}

static void on_slice(const zarray_t *pts,
                     const zarray_t *ranges,
                     const zarray_t *intensities,
                     uint8_t mode,
                     void *user)
{
    state_t *state = user;

    if (state->debug) {
        zarray_add_all(state->pts, pts);

        for (int i = 0; i < zarray_size(intensities); i++) {
            uint8_t intensity = 0;
            zarray_get(intensities, i, &intensity);
            float fintensity = intensity/255.0f;
            zarray_add(state->intensities, &fintensity);
        }
    }

    bool set_obs_or_slam = false;
    float d[3];
    for (int laseridx = 0; laseridx < zarray_size(pts)-1; laseridx++) {
        float *xyz = (float*)pts->data + 3*laseridx;
        float *next_xyz = (float*)pts->data + 3*(laseridx+1);

        float range = ((float*)ranges->data)[laseridx];
        float next_range = ((float*)ranges->data)[laseridx+1];

        // Skip null range points
        if (range <= .2 || next_range <= .2)
            continue;

        // Skip points that are too low
        if (next_xyz[2] < MIN_RELATIVE_HEIGHT)
            continue;

        // Skip points that are out of the map
        int ix = gridmap_get_index_x(state->gm, xyz[0]);
        int iy = gridmap_get_index_y(state->gm, xyz[1]);
        if (ix < 0 || iy < 0)
            continue;

        // Calculate slope between the two points
        for (int i = 0; i < 3; i++)
            d[i] = (next_xyz[i] - xyz[i]);

        float xy2 = d[0]*d[0] + d[1]*d[1];
        float slope_rad = acos(sqrt(xy2)/sqrt(xy2+d[2]*d[2]));

        // Detects objects exceeding max slope, in addition to some
        // heuristic methods for detecting dropoffs.
        if (slope_rad > MAX_SLOPE ||
            d[2] < -.1)
        {
            // obstacle only if not too high up
            if (next_xyz[2] <= MAX_RELATIVE_OBSTACLE_HEIGHT) {
                set_obs_or_slam = true;
                state->gm->data[iy*state->gm->width + ix] |= (uint8_t)GRID_FLAG_OBSTACLE;
            }

            // Slammable classifier
            if (slope_rad > SLAM_SLOPE && slope_rad < M_PI - SLAM_SLOPE) {
                set_obs_or_slam = true;
                state->gm->data[iy*state->gm->width + ix] |= (uint8_t)GRID_FLAG_SLAMMABLE;
            }
        }
    }

    // If an obstacle or slammable was detected, try fill-in
    if (set_obs_or_slam && state->last_slice) {
        for (int laseridx = 0; laseridx < zarray_size(pts); laseridx++) {
            // Try to connect points between scans. Our criteria are:
            // 1) Sufficiently close observations and
            // 2) Same grid map values
            float *xyz  = (float*)pts->data + 3*laseridx;
            float *xyz_ = (float*)state->last_slice->data + 3*laseridx;

            // Again, skip points out of range of caring
            if (xyz[2] < MIN_RELATIVE_HEIGHT)
                continue;

            // Skip points that obstacles or slammable
            uint8_t v = gridmap_get_value(state->gm, xyz[0], xyz[1]);
            if (!(v & (GRID_FLAG_OBSTACLE | GRID_FLAG_SLAMMABLE)))
                continue;

            for (int i = 0; i < 3; i++)
                d[i] = (xyz[i] - xyz_[i]);

            // Skip if points are sufficiently far apart
            float dist = floats_magnitude(d, 2);
            if (dist > MAX_FILL_IN_DIST || dist < state->gm->meters_per_pixel)
                continue;

            // Skip if map cells don't match hazard types
            uint8_t v_ = gridmap_get_value(state->gm, xyz_[0], xyz_[1]);
            if (v_ != v)
                continue;

            // Apply fill in
            int x0 = gridmap_get_index_x(state->gm, xyz[0]);
            int y0 = gridmap_get_index_y(state->gm, xyz[1]);
            int x1 = gridmap_get_index_x(state->gm, xyz_[0]);
            int y1 = gridmap_get_index_y(state->gm, xyz_[1]);
            int nsteps = (int)(dist/state->mpp);
            for (int i = 1; i < nsteps; i++) {
                double alpha = (double)i/nsteps;
                int x = round(x0*alpha + (1-alpha)*x1);
                int y = round(y0*alpha + (1-alpha)*y1);
                state->gm->data[y*state->gm->width + x] = v;
            }
        }
    }


    if (state->last_slice)
        zarray_destroy(state->last_slice);
    state->last_slice = zarray_copy(pts);
}

static void on_sweep(void *user)
{
    state_t *state = user;

    // Calculate observed free
    line_of_sight(state);

    // Debugging
    render_data(state);

    // Grid map publishing
    int64_t now = state->msg->utime;
    if (state->rmd_out.utime &&
        now - state->last_publish_utime > state->publish_delay_utime)
    {
        grid_map_t *crop = gridmap_crop(state->gm, true, GRID_FLAG_NONE);
        grid_map_t *encoded = grid_map_t_encode_gzip(crop);
        state->rmd_out.gridmap = *encoded;

        gps_t gps;
        if(-2 != interpolator_get(state->gps_interp, state->msg->utime, &gps) && (gps.status == GPS_T_GPS_STATUS_LOCK || gps.status == GPS_T_GPS_STATUS_DGPS_LOCK)) {
            state->rmd_out.latlon_deg[0] = gps.lat;
            state->rmd_out.latlon_deg[1] = gps.lon;
        } else {
            state->rmd_out.latlon_deg[0] = NAN;
            state->rmd_out.latlon_deg[1] = NAN;
        }

        mesh_t msg_out;
        msg_out.utime = state->msg->utime;
        msg_out.channel = state->map_channel;
        msg_out.echo = false;
        msg_out.src = state->robot_id;
        msg_out.dest = MESH_T_GROUND_ID;
        msg_out.broadcast = false;

        msg_out.nbytes = robot_map_data_t_encoded_size(&state->rmd_out);
        msg_out.bytes = calloc(msg_out.nbytes, sizeof(uint8_t));
        robot_map_data_t_encode(msg_out.bytes, 0, msg_out.nbytes, &state->rmd_out);

        lcm_publish(state->lcm, msg_out.channel, msg_out.bytes, msg_out.nbytes);

        if (state->rmd_out.utime &&
            now - state->mesh_publish_utime > state->mesh_delay_utime)
        {
            mesh_t_publish(state->lcm, state->tx_channel, &msg_out);
            state->mesh_publish_utime = now;
        }

        free(msg_out.bytes);

        gridmap_destroy(crop);
        gridmap_destroy(encoded);
        state->last_publish_utime = now;
    }

    // Init next map based on last robot pose.
    state->rmd_out.utime = state->msg->utime;
    memcpy(state->rmd_out.xyt_local, state->xyt_local, 3*sizeof(double));
    state->gm->x0 = state->xyt_local[0] - state->gm->width*state->gm->meters_per_pixel/2.0;
    state->gm->y0 = state->xyt_local[1] - state->gm->height*state->gm->meters_per_pixel/2.0;
    memset(state->gm->data, GRID_FLAG_NONE, state->gm->datalen);
}

void *process_velodyne_data(void* arg) {
    state_t *state = arg;

    int ret;
    int delay = 10; // Delay several messages

    pose_t pose_local;
    double xform_[16];
    float xform[16];

    if (1) {
        const zarray_t *xform_xyz = config_require_doubles(state->config, "velodyne.xyz_m");
        const zarray_t *xform_rpy = config_require_doubles(state->config, "velodyne.rpy_deg");

        double *xyz_ = (double*)xform_xyz->data;
        double *rpy_ = (double*)xform_rpy->data;
        double xyzrpy[6] = { xyz_[0],
                             xyz_[1],
                             xyz_[2],
                             to_radians(rpy_[0]),
                             to_radians(rpy_[1]),
                             to_radians(rpy_[2]) };

        doubles_xyzrpy_to_mat44(xyzrpy, xform_);
        for (int i = 0; i < 16; i++)
            xform[i] = (float)xform_[i];
    }

    while (1) {
        pthread_mutex_lock(&state->velodyne_msgs_mutex);
        if (zqueue_size(state->velodyne_msgs) <= delay) {
            pthread_mutex_unlock(&state->velodyne_msgs_mutex);
            timeutil_usleep(10000);
            continue;
        }
        zqueue_pop(state->velodyne_msgs, &state->msg);

        if (-2 == (ret = interpolator_get(state->pose_interp,
                                          state->msg->utime,
                                          &pose_local))) {
            pthread_mutex_unlock(&state->velodyne_msgs_mutex);
            raw_t_destroy(state->msg);
            timeutil_usleep(10000);
            continue;
        }

        pthread_mutex_unlock(&state->velodyne_msgs_mutex);

        // Setup map
        if (!state->map_init) {
            memcpy(state->rmd_out.xyt_local, state->xyt_local, 3*sizeof(double));
            state->gm->x0 = state->xyt_local[0] - state->gm->width*state->gm->meters_per_pixel/2.0;
            state->gm->y0 = state->xyt_local[1] - state->gm->height*state->gm->meters_per_pixel/2.0;
            memset(state->gm->data, GRID_FLAG_NONE, state->gm->datalen);
            state->map_init = true;
        }

        // Setup pose and sensor transform
        // XXX Later, we should make this take a few generic arguments
        // for positioning the sensor in the world beyond just pose.
        doubles_quat_xyz_to_xyt(pose_local.orientation,
                                pose_local.pos,
                                state->xyt_local);

        float M[16];
        float XYT[16];
        float fxyt[3] = {state->xyt_local[0],
                         state->xyt_local[1],
                         state->xyt_local[2]};
        floats_xyt_to_mat44(fxyt, XYT);
        floats_mat_AB(XYT, 4, 4,
                      xform, 4, 4,
                      M, 4, 4);

        april_velodyne_on_packet(state->velo,
                                 state->msg->buf,
                                 state->msg->len,
                                 M,
                                 on_slice,
                                 on_sweep,
                                 state);

        raw_t_destroy(state->msg);
    }
}

void init_state(state_t* state, getopt_t *gopt)
{
    state->debug = getopt_get_bool(gopt, "debug");
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(getopt_get_int(gopt, "port"),
                                       NULL,
                                       "index.html");
    printf("[INFO] viewer enabled on port %d\n", getopt_get_int(gopt, "port"));
    webvx_define_canvas(state->webvx, "velodyne_terrain_canvas",
                        on_create_canvas, on_destroy_canvas, state);

    if (!state->debug) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "debug-warn");
        vx_buffer_add_back(vb,
                           vxo_chain(vxo_matrix_scale(0.05),
                                     vxo_text(VXO_TEXT_ANCHOR_CENTER, "Please enable debug mode"),
                                     NULL),
                           NULL);
        vx_buffer_swap(vb);
    }
    state->pts = zarray_create(3*sizeof(float));
    state->intensities = zarray_create(1*sizeof(float));

    // Initialize LCM/message
    state->lcm = lcm_create(NULL);
    http_advertiser_create(state->lcm, getopt_get_int(gopt, "port"), "Velo2Map", "Velodyne to gridmap process");
    state->last_publish_utime = 0;
    state->publish_delay_utime = (int64_t)floor(1000000/getopt_get_double(gopt, "rate"));
    state->mesh_delay_utime = (int64_t)floor(1000000/getopt_get_double(gopt, "rate-mesh"));
    state->mpp = getopt_get_double(gopt, "mpp");
    state->line_nsteps = (int)(RAYCASTING_RANGE/state->mpp)*5 + 1;
    state->map_channel = strdup(getopt_get_string(gopt, "map-channel"));
    state->tx_channel = strdup(getopt_get_string(gopt, "tx-channel"));

    state->gm = gridmap_make_pixels(0, 0, GRID_MAP_SIZE, GRID_MAP_SIZE, state->mpp, GRID_FLAG_NONE, 0);
    state->map_init = false;

    // Raycasting init
    state->nrays = (int)ceil(2*M_PI/RAYCASTING_STEP);
    state->raycasting_offset_xy = calloc(2*state->nrays, sizeof(float));
    for (int i = 0; i < state->nrays; i++) {
        state->raycasting_offset_xy[2*i + 0] = cos(i*RAYCASTING_STEP)*RAYCASTING_RANGE;
        state->raycasting_offset_xy[2*i + 1] = sin(i*RAYCASTING_STEP)*RAYCASTING_RANGE;
    }

    state->pose_interp = interpolator_create(sizeof(pose_t), offsetof(pose_t, utime), 5.0, 100E3);
    interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_LINEAR, 3, offsetof(pose_t, pos));
    interpolator_add_field(state->pose_interp, INTERPOLATOR_DOUBLE_QUAT, 4, offsetof(pose_t, orientation));

    state->robot_id = getopt_get_int(gopt, "robot-id");

    state->gps_interp = interpolator_create(sizeof(gps_t), offsetof(gps_t, host_utime), 5.0, 1 * 1000 * 1000);
    interpolator_add_field(state->gps_interp, INTERPOLATOR_DOUBLE_LINEAR, 1, offsetof(gps_t, lat));
    interpolator_add_field(state->gps_interp, INTERPOLATOR_DOUBLE_LINEAR, 1, offsetof(gps_t, lon));

    // Velodyne msg queue
    state->velodyne_msgs = zqueue_create(sizeof(raw_t*));
    zqueue_ensure_capacity(state->velodyne_msgs, 100);
    if(!state->lcm) {
        printf("[ERROR] impossible to initialize LCM environment... quitting!");
        exit(-1);
    }

    state->velo = april_velodyne_create();

    // Initialize robot-specific config
    state->config = config_create_path(getopt_get_string(gopt, "config"));
}

int main(int argc, char *argv[]) {
    setlinebuf(stderr);
    setlinebuf(stdout);

    signal(SIGINT, signal_handler);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'd', "debug", 0, "Debugging visualization");
    getopt_add_int(gopt, 'p', "port", "8222", "vx port");
    getopt_add_string(gopt, 'c', "config", magic_util_config("robot-concat.config"), "Config file");
    getopt_add_string(gopt, '\0', "lidar-channel", "VELODYNE_DATA", "Velodyne channel");
    getopt_add_string(gopt, '\0', "pose-channel", "POSE", "Pose channel");
    getopt_add_string(gopt, '\0', "map-channel", "ROBOT_MAP_DATA", "Map data channel");
    getopt_add_string(gopt, '\0', "tx-channel", "MESH_TX", "Mesh transmit channel");
    getopt_add_double(gopt, '\0', "rate", "5.5", "Rate at which to publish robot_map_data_t");
    getopt_add_double(gopt, '\0', "mpp", "0.05", "Meters per pixel");
    getopt_add_double(gopt, '\0', "rate-mesh", "1.01", "Rate at which to publish robot_map_data_t to the mesh");
    getopt_add_string(gopt, '\0', "gps-channel", "GPS", "gps data channel (input)");
    getopt_add_int(gopt, 'r', "robot-id", magic_util_id_string(), "Robot ID");

    if (!getopt_parse(gopt, argc, argv, 0)) {
        fprintf(stderr, "ERR: getopt_parse\n");
        return -1;
    }

    if (getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    // init state
    state_t* state = calloc(1, sizeof(state_t));
    init_state(state, gopt);

    // create thread that processes velodyne point clouds
    pthread_t tid;
    int err = pthread_create(&tid, NULL, &process_velodyne_data, state);
    if(err != 0) {
        printf("[ERROR] can't create thread to process velodyne point clouds [%s]\n",
               strerror(err));
        return -1;
    }

    // subscribe to velodyne data channel
    raw_t_subscribe(state->lcm, getopt_get_string(gopt, "lidar-channel"), velodyne_callback, state);
    pose_t_subscribe(state->lcm, getopt_get_string(gopt, "pose-channel"), pose_handler, state);
    gps_t_subscribe(state->lcm, getopt_get_string(gopt, "gps-channel"), on_gps, state);

    // run the loop
    while(1) {
        lcm_handle(state->lcm);
    }

    zqueue_destroy(state->velodyne_msgs);
    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
