#include <signal.h>
#include <float.h>

#include "drive_to_wp_state.h"
#include "vfh_star.h"
#include "gui.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "common/config.h"
#include "common/getopt.h"
#include "common/time_util.h"
#include "velodyne_to_map2/gridmap2.h"

// changed by SIG_INT only
bool continue_running = true;

double dist_to_dest(drive_to_wp_state_t *state)
{
    double diff_x = state->last_cmd->xyt[0] - state->xyt[0];
    double diff_y = state->last_cmd->xyt[1] - state->xyt[1];
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    return sqrt(dist_sq);
}

bool has_reached_dest(drive_to_wp_state_t *state)
{
    if (!state->last_pose) {
        return false;
    }

    double diff_x = state->last_cmd->xyt[0] - state->xyt[0];
    double diff_y = state->last_cmd->xyt[1] - state->xyt[1];
    double dist_sq = diff_x * diff_x + diff_y * diff_y;
    return dist_sq <= state->last_cmd->achievement_dist * state->last_cmd->achievement_dist;
}

void receive_pose(const lcm_recv_buf_t *rbuf, const char *channel,
                  const pose_t *pose, void *user)
{
    drive_to_wp_state_t *state = user;
    if (!state->debugging && state->last_pose && pose->utime <= state->last_pose->utime) {
        return;
    }
    if (state->last_pose) {
        pose_t_destroy(state->last_pose);
    }
    state->last_pose = pose_t_copy(pose);

    doubles_quat_xyz_to_xyt(pose->orientation, pose->pos, state->xyt);

    double q_inv[4];
    doubles_quat_inverse(pose->orientation, q_inv);

    double v_body[3];
    doubles_quat_rotate(q_inv, pose->vel, v_body);

    state->forward_vel = v_body[0];
}

void receive_robot_map_data(const lcm_recv_buf_t *rbuf, const char *channel,
                            const robot_map_data_t *msg, void *user)
{
    drive_to_wp_state_t *state = user;
    if (!state->debugging && state->last_map_data && msg->utime <= state->last_map_data->utime) {
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

void receive_waypoint_cmd(const lcm_recv_buf_t *rbuf, const char *channel,
                          const waypoint_cmd_t *msg, void *user)
{
    drive_to_wp_state_t *state = user;
    if (!state->debugging && state->last_cmd && msg->utime <= state->last_cmd->utime) {
        return;
    }
    if (state->last_cmd) {
        waypoint_cmd_t_destroy(state->last_cmd);
    }
    if (isnan(msg->xyt[0])) {
        state->last_cmd = NULL;
    } else {
        state->last_cmd = waypoint_cmd_t_copy(msg);
    }
}

double constrain(double val, double min_val, double max_val)
{
    if (val < min_val) {
        return min_val;
    } else if (val > max_val) {
        return max_val;
    }
    return val;
}

static void rasterize_poly_line(int *buff_x0, int *buff_x1, int startX, int startY, int endX, int endY)
{
    // Bresenham's Line Drawing, as applied to rasterizing a convex polygon
    int cx = startX;
    int cy = startY;

    int dx = abs(endX - startX);
    int dy = abs(endY - startY);

    int sx = startX < endX ? 1 : -1;
    int sy = startY < endY ? 1 : -1;

    int err = dx - dy;

    for (int n = 0; n < 1000; n++) {
        if (sx < 0) {
            buff_x0[cy] = cx;
        } else if (sx > 0) {
            buff_x1[cy] = cx;
        } else {
            if (buff_x0[cy] == -1) {
                buff_x0[cy] = cx;
            } else {
                buff_x0[cy] = min(buff_x0[cy], cx);
            }
            if (buff_x1[cy] == -1) {
                buff_x1[cy] = cx;
            } else {
                buff_x1[cy] = max(buff_x0[cy], cx);
            }
        }

        if ((cx == endX) && (cy == endY)) {
            return;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            cx = cx + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            cy = cy + sy;
        }
    }
}

bool obstacle_in_region(drive_to_wp_state_t *state,
                        double left_dist, double right_dist, double forward_dist, double backward_dist)
{
    grid_map_t *gm = state->last_grid_map;
    if (!gm) {
        return true;
    }

    double cos_theta = cos(state->xyt[2]);
    double sin_theta = sin(state->xyt[2]);

    double xs[4];
    double ys[4];

    xs[0] = state->xyt[0] - left_dist * sin_theta - backward_dist * cos_theta;
    ys[0] = state->xyt[1] - left_dist * cos_theta - backward_dist * sin_theta;

    xs[1] = state->xyt[0] - left_dist * sin_theta + forward_dist * cos_theta;
    ys[1] = state->xyt[1] - left_dist * cos_theta + forward_dist * sin_theta;

    xs[2] = state->xyt[0] + right_dist * sin_theta + forward_dist * cos_theta;
    ys[2] = state->xyt[1] + right_dist * cos_theta + forward_dist * sin_theta;

    xs[3] = state->xyt[0] + right_dist * sin_theta - backward_dist * cos_theta;
    ys[3] = state->xyt[1] + right_dist * cos_theta - backward_dist * sin_theta;

    int ixs[4];
    int iys[4];
    int minIY = -1;
    int maxIY = -1;
    for (int i = 0; i < 4; i++) {
        ixs[i] = gridmap_get_index_x(gm, xs[i]);
        iys[i] = gridmap_get_index_y(gm, ys[i]);
        if (minIY == -1 || iys[i] < minIY) {
            minIY = iys[i];
        }
        if (maxIY == -1 || iys[i] > maxIY) {
            maxIY = iys[i];
        }
    }

    // rectangle rasterization: https://stackoverflow.com/questions/10061146/how-to-rasterize-rotated-rectangle-in-2d-by-setpixel
    int buff_x0[gm->height];
    int buff_x1[gm->height];
    for (int i = 0; i < gm->height; i++) {
        buff_x0[i] = -1;
        buff_x1[i] = -1;
    }

    rasterize_poly_line(buff_x0, buff_x1, ixs[0], iys[0], ixs[1], iys[1]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[1], iys[1], ixs[2], iys[2]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[2], iys[2], ixs[3], iys[3]);
    rasterize_poly_line(buff_x0, buff_x1, ixs[3], iys[3], ixs[0], iys[0]);

    for (int y = minIY; y <= maxIY; y++) {
        for (int x = buff_x0[y]; x <= buff_x1[y]; x++) {
            if (!(gm->data[y * gm->width + x] & GRID_FLAG_TRAVERSABLE)) {
                return true;
            }
        }
    }

    return false;
}

bool obstacle_ahead(drive_to_wp_state_t *state)
{
    double left_dist = state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_speed = max(0, state->forward_vel);
    double forward_dist = max(state->min_forward_distance, state->min_forward_per_mps * forward_speed);
    double backward_dist = 0;
    return obstacle_in_region(state, left_dist, right_dist, forward_dist, backward_dist);
}

bool obstacle_behind(drive_to_wp_state_t *state)
{
    double left_dist = state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_dist = 0;
    double backward_speed = max(0, -state->forward_vel);
    double backward_dist = max(state->min_forward_distance, state->min_forward_per_mps * backward_speed);
    return obstacle_in_region(state, left_dist, right_dist, forward_dist, backward_dist);
}

bool obstacle_by_sides(drive_to_wp_state_t *state)
{
    double left_dist = state->min_side_distance + state->vehicle_width / 2;
    double right_dist = left_dist;
    double forward_dist = state->min_forward_distance;
    double backward_dist = state->min_forward_distance;
    return obstacle_in_region(state, left_dist, right_dist, forward_dist, backward_dist);
}

void apply_safety_limits(drive_to_wp_state_t *state, double *forward_motor, double *turning_motor)
{
    grid_map_t *gm = state->last_grid_map;
    if (!gm) {
        *forward_motor = 0;
        *turning_motor = 0;
        return;
    }

    if (state->has_obstacle_ahead && *forward_motor > 0) {
        *forward_motor = 0;
    }
    if (state->has_obstacle_behind && *forward_motor < 0) {
        *forward_motor = 0;
    }
    if (state->has_obstacle_by_sides) {
        *turning_motor = 0;
    }
}

void update_control(drive_to_wp_state_t *state)
{
    // make forward dist a function of velocity
    state->has_obstacle_ahead = obstacle_ahead(state);
    state->has_obstacle_behind = obstacle_behind(state);
    state->has_obstacle_by_sides = obstacle_by_sides(state);
    // if (state->stopped_for_obstacle) {
    //     diff_drive_t motor_cmd = {
    //         .utime = utime_now(),
    //         .left = 0,
    //         .left_enabled = false,
    //         .right = 0,
    //         .right_enabled = false
    //     };
    //     diff_drive_t_publish(state->lcm, "DIFF_DRIVE", &motor_cmd);
    //     return;
    // }

    if (!state->last_cmd) {
        return;
    }

    // printf("\nStarting min_turning_r comparison...\n");
    waypoint_cmd_t *cmd = state->last_cmd;
    vfh_star_result_t *best_result = NULL;
    double min_turning_r = 0;
    for (double turn_r = 0; turn_r < 1.0; turn_r += 0.1) {
        vfh_star_result_t *result = vfh_star_update(state, cmd->xyt[0], cmd->xyt[1], turn_r);
        // printf("turn_r: %.1f cost: %.2f\n", turn_r, result->cost);
        if (result && (!best_result || result->cost <= best_result->cost)) {
            min_turning_r = turn_r;
            if (best_result) {
                vfh_star_result_destroy(best_result);
            }
            best_result = result;
        } else if (result) {
            vfh_star_result_destroy(result);
        }
    }
    // printf("Choose turn_r: %.1f cost: %.2f\n", min_turning_r, best_result->cost);

    double target_heading = state->chosen_direction; // from last time...
    if (best_result) {
        target_heading = best_result->target_heading;
    }

    render_vfh_star(state, best_result);
    vfh_star_result_destroy(best_result);

    double slowest_turn_r = 1.0; // slower turning allows faster forward movement
    double forward_r_slowdown = sqrt(min_turning_r / slowest_turn_r);

    double forward_motor = 0;
    double turning_motor = 0;

    double dist = dist_to_dest(state);
    if (dist > cmd->achievement_dist) {
        double heading_err = mod2pi(target_heading - state->xyt[2]);

        // dot product of the vfh* direction and our current direction
        // (equivalent to cosine of the error)
        // determines division of forward and turning speed
        forward_motor = cos(heading_err);//max(0, cos(heading_err));

        bool backwards_best = fabs(heading_err) > M_PI / 2;
        if (backwards_best) {
            heading_err = mod2pi(heading_err - M_PI);
        }

        // sqrt(1 - sq(f_vel)) is effectively the sin, corrected for only going forwards
        //turning_motor = sin(heading_err);//copysign(sqrt(1 - sq(forward_motor)), heading_err);
        // and then we also scale these by the error in each with appropriate limits
        forward_motor *= constrain(0.5 * dist, 0.1, 0.4);
        forward_motor *= forward_r_slowdown;
        turning_motor = copysign(constrain(0.9 * fabs(heading_err), 0.1, 0.2), heading_err); // 0.25, 0.4

        if (fabs(forward_motor) < 0.15) {
            turning_motor = copysign(fabs(turning_motor) + 0.25, turning_motor);
        }

        // printf("Heading err: %.2f forward: %.2f turning: %.2f ", heading_err, forward_motor, turning_motor);
    }

    // apply low-pass smoothing to motor movement
    uint64_t now = utime_now();
    if (state->last_motor_utime == 0) {
        state->last_motor_utime = now;
    }

    double dt = (now - state->last_motor_utime) * 1e-6;
    double alpha_factor = 2 * M_PI * dt * state->motor_low_pass_f;
    double alpha = alpha_factor / (alpha_factor + 1);
    state->last_forward = (1 - alpha) * state->last_forward + alpha * forward_motor;
    state->last_turning = (1 - alpha) * state->last_turning + alpha * turning_motor;

    // limits which velocities can be non-zero based on obstacles
    apply_safety_limits(state, &state->last_forward, &state->last_turning);
    if (state->last_forward == 0 && state->last_turning == 0) {
        state->stopped_for_obstacle = true;
    } else {
        state->stopped_for_obstacle = false;
    }

    // printf("filtered forward: %.2f turning %.2f\n", state->last_forward, state->last_turning);

    state->last_motor_utime = now;

    double left_motor = state->last_forward - state->last_turning;
    double right_motor = state->last_forward + state->last_turning;

    // double target_heading = atan2(cmd->xyt[1] - state->xyt[1], cmd->xyt[0] - state->xyt[0]);
    // double heading_err = mod2pi(target_heading - state->xyt[2]);
    // if (fabs(heading_err) > HEADING_THRESH) {
    //     double mag = constrain(0.9 * heading_err, 0.25, 0.4);
    //     left_motor = -copysign(mag, heading_err);
    //     right_motor = -left_motor;
    // } else {
    //     double dist = dist_to_dest(state);
    //     if (dist > cmd->achievement_dist) {
    //         double mag = constrain(0.5 * dist, 0.2, 0.6);
    //         left_motor = mag;
    //         right_motor = mag;
    //     }
    // }

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

void signal_handler(int v)
{
    continue_running = false;
}

int main(int argc, char **argv)
{
    struct sigaction signal_action = { 0 };
    signal_action.sa_handler = signal_handler;
    sigaction(SIGINT, &signal_action, NULL);

    setlinebuf(stdout);
    setlinebuf(stderr);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_string(gopt, '\0', "pose-channel", "POSE", "pose_t channel");
    getopt_add_string(gopt, 'c', "config", "./config/robot.config", "Config file");
    getopt_add_double(gopt, '\0', "hz", "100", "Config file");
    getopt_add_bool(gopt, '\0', "debug", 0, "Debugging mode");
    if (!getopt_parse(gopt, argc, argv, true) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(getopt_get_bool(gopt, "help") ? EXIT_SUCCESS : EXIT_FAILURE);
    }

    double control_update_hz = getopt_get_double(gopt, "hz");
    require_value_nonnegative(control_update_hz, "hz");

    drive_to_wp_state_t state = { 0 };

    state.debugging = getopt_get_bool(gopt, "debug");

    state.lcm = lcm_create(NULL);
    if (!state.lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }

    char *config_path = str_expand_envs(getopt_get_string(gopt, "config"));
    config_t *config = config_create_path(config_path);
    free(config_path);
    if (!config) {
        fprintf(stderr, "ERR: Unable to open config file: %s\n", getopt_get_string(gopt, "config"));
        exit(EXIT_FAILURE);
    }

    state.vehicle_width = config_require_double(config, "robot.geometry.width");
    require_value_nonnegative(state.vehicle_width, "robot.geometry.width");

    state.motor_low_pass_f = config_require_double(config, "drive_to_wp.motor_low_pass_freq");
    require_value_nonnegative(state.motor_low_pass_f, "drive_to_wp.motor_low_pass_freq");

    state.min_side_distance = config_require_double(config, "drive_to_wp.min_side_distance");
    require_value_nonnegative(state.min_side_distance, "drive_to_wp.min_side_distance");

    state.min_forward_distance = config_require_double(config, "drive_to_wp.min_forward_distance");
    require_value_nonnegative(state.min_forward_distance, "drive_to_wp.min_forward_distance");

    state.min_forward_per_mps = config_require_double(config, "drive_to_wp.min_forward_per_mps");
    require_value_nonnegative(state.min_forward_per_mps, "drive_to_wp.min_forward_per_mps");

    pose_t_subscribe(state.lcm, "POSE", receive_pose, &state);
    robot_map_data_t_subscribe(state.lcm, "ROBOT_MAP_DATA", receive_robot_map_data, &state);
    waypoint_cmd_t_subscribe(state.lcm, "WAYPOINT_CMD", receive_waypoint_cmd, &state);

    initialize_vfh_star(&state, config);
    gui_init(&state);

    timeutil_rest_t *timer = timeutil_rest_create();
    while (continue_running) {
        lcm_handle_async(state.lcm);

        update_control(&state);
        render_gui(&state);

        timeutil_sleep_hz(timer, control_update_hz);
    }
    timeutil_rest_destroy(timer);

    vfh_release_state(&state);
    if (state.last_map_data) {
        robot_map_data_t_destroy(state.last_map_data);
    }
    if (state.last_grid_map) {
        grid_map_t_destroy(state.last_grid_map);
    }
    if (state.last_pose) {
        pose_t_destroy(state.last_pose);
    }
    if (state.last_cmd) {
        waypoint_cmd_t_destroy(state.last_cmd);
    }
    config_destroy(config);
    getopt_destroy(gopt);
    lcm_destroy(state.lcm);
    return 0;
}
