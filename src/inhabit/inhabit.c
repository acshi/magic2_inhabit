#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <sys/select.h>

#include "common/doubles.h"
#include "common/time_util.h"
#include "common/magic_util.h"
#include "common/gridmap_util.h"
#include "common/lcm_handle_async.h"
#include "scanmatch/scanmatch.h"
#include "lcm/lcm.h"
#include "lcmtypes/diff_drive_t.h"
#include "lcmtypes/robot_command_t.h"
#include "lcmtypes/robot_task_t.h"
#include "lcmtypes/robot_map_data_t.h"
#include "lcmtypes/grid_map_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/lcmdoubles_t.h"
#include "lcmtypes/waypoint_cmd_t.h"
#include "velodyne_to_map2/gridmap2.h"

// in meters
#define DEAD_BAND 0.05

#define SCANS_TO_AVG 16
#define SCAN_THRESHOLD 5

typedef struct {
    lcm_t *lcm;
    pose_t *last_pose;
    robot_map_data_t *last_map_data;
    grid_map_t *last_grid_map;

    double zero_xyt[3];
    double robot_xyt[3];

    int state_num;

    sm_model_data_t *place_model;
    image_u8_t *place_map;
    bool is_scanning_place;
    int scan_i;
    long last_scan_utime;

    FILE *debug_file;
} inhabit_t;

#define CONTROL_UPDATE_MS 15

bool has_reached_pos(inhabit_t *state, double *xy) {
    if (!state->last_pose) {
        return false;
    }

    double diff_x = fabs(xy[0] - state->robot_xyt[0]);
    double diff_y = fabs(xy[1] - state->robot_xyt[1]);
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    printf("robot: %.3f, %.3f, %.3f, target: %.3f, %.3f Dist to waypoint: %7.4f\r", state->robot_xyt[0], state->robot_xyt[1], state->robot_xyt[2], xy[0], xy[1], sqrt(dist_sq));
    fflush(stdout);
    return dist_sq <= DEAD_BAND * DEAD_BAND;
}

bool drive_to(inhabit_t *state, double* target_xy) {
    if (has_reached_pos(state, target_xy)) {
        return true;
    }

    double target_xyt[3] = {target_xy[0], target_xy[1], 0};
    double target_with_zero[3];
    doubles_xyt_mul(state->zero_xyt, target_xyt, target_with_zero);

    //printf("Driving to %.3f, %.3f, %.3f transformed by %.3f, %.3f, %.3f to get %.3f, %.3f, %.3f\n", target_xyt[0], target_xyt[1], target_xyt[2],
    //    state->zero_xyt[0], state->zero_xyt[1], state->zero_xyt[2], target_with_zero[0], target_with_zero[1], target_with_zero[2]);

    waypoint_cmd_t cmd = {
        .utime = utime_now(),
        .xyt = {target_with_zero[0], target_with_zero[1], NAN},
        .achievement_dist = DEAD_BAND
    };
    waypoint_cmd_t_publish(state->lcm, "WAYPOINT_CMD", &cmd);

    //fprintf(state->debug_file, "WAYPOINT_CMD: utime: %ld x: %.3f y: %.3f\n", cmd.utime, cmd.xyt[0], cmd.xyt[1]);

    return false;
}

void receive_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_t *msg, void *user)
{
    //printf("Receiving POSE message %.3f seconds after sent\n", (utime_now() - msg->utime) / 1e6);

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

    // state->robot_xyt[0] = received_xyt[0] - state->zero_xyt[0];
    // state->robot_xyt[1] = received_xyt[1] - state->zero_xyt[1];
    // state->robot_xyt[2] = mod2pi(received_xyt[2] - state->zero_xyt[2]);
    doubles_xyt_inv_mul(state->zero_xyt, received_xyt, state->robot_xyt);

    //fprintf(state->debug_file, "POSE: utime: %ld x: %.3f y: %.3f t: %.3f\n", msg->utime, state->robot_xyt[0], state->robot_xyt[1], state->robot_xyt[2]);

    //printf("Received %.3f, %.3f, %.3f and transformed by %.3f, %.3f, %.3f to get %.3f, %.3f, %.3f\n", received_xyt[0], received_xyt[1], received_xyt[2],
    //    state->zero_xyt[0], state->zero_xyt[1], state->zero_xyt[2], state->robot_xyt[0], state->robot_xyt[1], state->robot_xyt[2]);
}

void receive_robot_map_data(const lcm_recv_buf_t *rbuf, const char *channel,
                            const robot_map_data_t *msg, void *user)
{
    inhabit_t *state = user;
    if (state->last_map_data && msg->utime <= state->last_map_data->utime) {
        return;
    }
    if (state->last_map_data) {
        robot_map_data_t_destroy(state->last_map_data);
    }
    state->last_map_data = robot_map_data_t_copy(msg);
    if (state->last_grid_map) {
        grid_map_t_destroy(state->last_grid_map);
    }
    state->last_grid_map = gridmap_decode_and_copy(&msg->gridmap);
}

void log_diff_drive(const lcm_recv_buf_t *rbuf, const char *channel,
                    const diff_drive_t *msg, void *user)
{
    //printf("Receiving DIFF_DRIVE message %.3f seconds after sent\n", (utime_now() - msg->utime) / 1e6);

    //inhabit_t *state = user;
    //fprintf(state->debug_file, "DIFF_DRIVE: utime: %ld left: %.3f right: %.3f\n", msg->utime, msg->left, msg->right);
}

void drive_in_square(inhabit_t *state) {
    double way_points[4][2] = {{1, 0}, {1, 1}, {0, 1}, {0, 0}};

    if (drive_to(state, way_points[state->state_num])) {
        printf("\nGot to corner at (%.3f, %.3f)\n", way_points[state->state_num][0], way_points[state->state_num][1]);
        state->state_num++;
        if (state->state_num >= 4) {
            state->state_num = 0;
        }
    }
}

void reset_odometry(inhabit_t *state) {
    if (state->last_pose) {
        doubles_quat_xyz_to_xyt(state->last_pose->orientation, state->last_pose->pos, state->zero_xyt);
        //printf("Using zero pose of: %.3f, %.3f, %.4f\n", state->zero_xyt[0], state->zero_xyt[1], state->zero_xyt[2]);
    }
    state->robot_xyt[0] = 0;
    state->robot_xyt[1] = 0;
    state->robot_xyt[2] = 0;
}

/*void take_scan() {
    // ROBOT_MAP_DATA looks like an image,
    // we take robot_map_data.gridmap, which will likely need to be decoded
    grid_map_t *gridmap_decode_and_copy (const grid_map_t *orig);
    // gridmap x0, y0 are in meters, local coordinates, the bottom left corner of the map itself
    // robot_map_data.pose gives us the robot's location in local coordinates as well.
    // the byte buffer is essentially an enum value at each place. The only one we care about is GRID_VAL_SLAMMABLE.
    // a good project would be to use flags instead of an enum... or just make more enum values like SLAMMABLE_TRAVERSABLE

    // has flags for points that are slammable
    // The points can be made into an image, as in the sm_model_data_t,
    // or a list of points as in sm_points_data_t
    // the list of points is easier to rotate.
    // sm_model_data_t has multiple sm_model_t images, all the same data, just with decimated versions
    sm_model_data_t place_model = sm_model_data_create(image_u8_t *im,
                                          int32_t x0, int32_t y0,
                                          float meters_per_pixel,
                                          int nresolutions);

    // for a scan match, we will specify limits of x, y, and theta min and maxes
    // as well as a thata step size. x and y step sizes are determined by the granularilty
    // of the highest resolution level model. Decimate early if you want less steps in x and y.
    // You also set the number of decimation levels. Scan match first looks at the grossest resolution
    // Score is the average pixel value that your points fell on.
    // Grosser levels will always give scores >= more detailed levels
    // So a match at the lowest tier means you can throw away all other possible
    // levels with worse scores because they can't get better

    // Don't use mean and inf fields (set to NULL) because they are not rigorous
    // They are intended to be a prior, and add a penalty to the score: makes scores harder to interpret...

    sm_search_t *scan_match_search = sm_search_create();

    // pixels_data comes in meters

    sm_search_handle_t *search_handle = sm_search_add(scan_match_search,
                                      sm_points_data_t *points_data, sm_model_data_t *model_data,
                                      int32_t tx0, int32_t tx1, int32_t ty0, int32_t ty1, // search bounds x and y min and max -- in pixels!!! / meters_per_pixel for pixels.
                                      float rad0, float rad1, float radstep, // theta/angle min max step
                                      float scale, NULL, NULL, float minscore); // scale should be 1/num points
                                      // minscore between 0 and 255.

    sm_result_t *search_result = sm_search_run(scan_match_search);

    // Use 1 degree for theta step, and then use hill climbing to get a finer degree of accuracy

    typedef struct sm_hillclimb_params sm_hillclimb_params_t;
    struct sm_hillclimb_params {
        double initial_step_sizes[3]; // initial step size in x, y, theta, meters_per_pixel/2 for x and y and radstep / 2.
        double step_size_shrink_factor; // how much to shrink step size by. (e.g. 0.5)
        int max_step_size_shrinks; // how many times, ~8-10

        int maxiters; // ~500 good. probably 1-100 at worst
    };

    sm_hillclimb_result_t hillclimb_result = *sm_hillclimb(sm_points_data_t *points_data, sm_model_data_t *model_data,
                                        double xyt0[3], sm_hillclimb_params_t *params,
                                        float scale, NULL, NULL);

    sm_hillclimb_result_destroy(hillclimb_result);

    sm_result_destroy(search_result);
}*/

void process_scan(inhabit_t *state) {
    if (!state->is_scanning_place || !state->last_grid_map ||
        state->last_grid_map->utime <= state->last_scan_utime) {
        return;
    }
    grid_map_t *gm = state->last_grid_map;
    if (!state->place_map) {
        state->place_map = image_u8_create(gm->width, gm->height);
    }
    image_u8_t *im = state->place_map;

    for (int y = 0; y < gm->height; y++) {
        for (int x = 0; x < gm->width; x++) {
            if (gm->data[y * gm->width + x] & GRID_FLAG_SLAMMABLE) {
                im->buf[y * im->stride + x]++;
            }
        }
    }

    state->scan_i++;
    if (state->scan_i >= SCANS_TO_AVG) {
        // done... threshold the image
        for (int y = 0; y < gm->height; y++) {
            for (int x = 0; x < gm->width; x++) {
                if (im->buf[y * im->stride + x] > SCAN_THRESHOLD) {
                    im->buf[y * im->stride + x] = 0xff;
                } else {
                    im->buf[y * im->stride + x] = 0;
                }
            }
        }

        if (state->place_model) {
            sm_model_data_destroy(state->place_model);
        }
        state->place_model = sm_model_data_create(im,
                                  (int32_t)gm->x0,
                                  (int32_t)gm->y0,
                                  (float)gm->meters_per_pixel,
                                  5);

        state->is_scanning_place = false;
        state->scan_i = 0;
        memset(&im->buf, 0, im->height * im->stride);
    }

    state->last_scan_utime = gm->utime;
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
    state->lcm = lcm_create(NULL);
    if (!state->lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    state->debug_file = fopen("debug.csv", "w");
    if (!state->debug_file) {
        fprintf(stderr, "Failed to open debug.csv. Aborting.\n");
        return 1;
    }

    pose_t_subscribe(state->lcm, "POSE", receive_pose, state);
    robot_map_data_t_subscribe(state->lcm, "ROBOT_MAP_DATA", receive_robot_map_data, state);
    diff_drive_t_subscribe(state->lcm, "DIFF_DRIVE", log_diff_drive, state);

    printf("Inhabit Running!\n");

    bool started = false;
    printf("Hit Enter to start\n");

    while (1) {
        lcm_handle_async(state->lcm);
        if (!started && kbhit()) {
            started = true;
            printf("Attempting to drive in a counter-clockwise square...\n");
            reset_odometry(state);
        }
        if (started) {
            drive_in_square(state);
            //process_scan(state);
        }
        nanosleep(&(struct timespec){0, (CONTROL_UPDATE_MS * 1e6)}, NULL);
    }

    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
