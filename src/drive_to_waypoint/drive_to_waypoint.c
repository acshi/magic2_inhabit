#include <signal.h>
#include <float.h>

#include "drive_to_wp_state.h"
#include "collision.h"
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

void apply_safety_limits(drive_to_wp_state_t *state, double *forward_motor, double *turning_motor)
{
    grid_map_t *gm = state->last_grid_map;
    if (!gm) {
        *forward_motor = 0;
        *turning_motor = 0;
            return;
    }

    if (state->is_blocked_ahead && *forward_motor > 0 && state->obstacle_ahead_slowdown == 0) {
        *forward_motor *= state->obstacle_ahead_slowdown;
    }
    if (state->is_blocked_behind && *forward_motor < 0 && state->obstacle_behind_slowdown == 0) {
        *forward_motor *= state->obstacle_behind_slowdown;
    }
    if (state->is_blocked_by_sides && state->obstacle_by_sides_slowdown == 0) {
        *turning_motor *= state->obstacle_by_sides_slowdown;
    }
}

double seconds() {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) {
        fprintf(stderr, "Retrieving system time failed.\n");
        exit(1);
    }
    return now.tv_sec + (double)now.tv_nsec * 1e-9;
}

// we would like to avoid changing heading much if we can
// since we know what achievement distance is okay for our target
// we can actually drive to a slightly difference target if that
// means the robot would not have to anymore
void calc_easier_target(drive_to_wp_state_t *state, double *easier_xy, bool *is_easier)
{
    double *true_target = state->last_cmd->xyt;
    // allow only half the achievement dist on either side
    // if we are less conservative we might drive and then miss the circle
    // of radius achievement_dist around the true target
    double wiggle_room = state->last_cmd->achievement_dist / 2;

    double heading = state->xyt[2];

    double dx = true_target[0] - state->xyt[0];
    double dy = true_target[1] - state->xyt[1];
    // The distance along current heading to the point closest to true target
    double forward_dist = dx * cos(heading) + dy * sin(heading);

    double forward_target[2] = { state->xyt[0] + forward_dist * cos(heading),
                                 state->xyt[1] + forward_dist * sin(heading) };

    double forward_error = sq(true_target[0] - forward_target[0]) +
                           sq(true_target[1] - forward_target[1]);

    if (forward_error <= sq(wiggle_room)) {
        easier_xy[0] = forward_target[0];
        easier_xy[1] = forward_target[1];
        *is_easier = true;
        // printf("Using easier target! \n");
    } else {
        *is_easier = false;
        easier_xy[0] = true_target[0];
        easier_xy[1] = true_target[1];
        // printf("Couldn't use easier target, error: %6.3f true x: %6.3f true y: %6.3f fwd x: %6.3f fwd y: %6.3f\n",
        //             forward_error, true_target[0], true_target[1], forward_target[0], forward_target[1]);
    }
}

void update_control_vfh(drive_to_wp_state_t *state, bool *requires_nonturning_solution)
{
    // vfh* is more expensive, only recompute every X control iterations
    if ((state->control_iteration % state->vfh_update_every) == 0) {
        // double start = seconds();
        *requires_nonturning_solution = false;

        double easier_xy[2];
        bool is_easier;
        calc_easier_target(state, easier_xy, &is_easier);

        // Report the size of the robot to VFH* depending on how much we might need to turn
        // since VFH* assumes a constant robot radius
        double command_angle = atan2(easier_xy[1] - state->xyt[1], easier_xy[0] - state->xyt[0]);
        double angle_off = min(M_PI / 4, fabs(mod2pi(command_angle - state->xyt[2])));
        double min_diam = state->vehicle_width;
        double max_diam = sqrt(sq(state->vehicle_width) + sq(state->vehicle_length));
        double effective_robot_diam = min_diam + (max_diam - min_diam) * sin(angle_off);

        vfh_star_result_t *best_result = NULL;
        double turning_r_options[] = {0, 0.1, 0.2, 0.4, 0.7, 1.0};
        int r_options_n = sizeof(turning_r_options) / sizeof(double);
        double best_turning_r = 0;
        for (int turn_r_i = 0; turn_r_i < r_options_n; turn_r_i++) {
            double turn_r = turning_r_options[turn_r_i];
            vfh_star_result_t *result =
                vfh_star_update(state, easier_xy[0], easier_xy[1],
                                turn_r, effective_robot_diam);
            // if (result) {
            //     printf("turn_r: %.1f last_dir_i: %d dir_i: %d cost: %.2f\n", turn_r, state->chosen_directions_i[0], result->target_headings_i[0], result->cost);
            // }

            result->cost += (1.0 - turn_r) * state->cost_turning_radius;

            if (result && (!best_result || result->cost <= best_result->cost)) {
                if (best_result) {
                    vfh_star_result_destroy(best_result);
                }
                best_result = result;
                best_turning_r = turn_r;
            } else if (result) {
                vfh_star_result_destroy(result);
            }
        }

        if (!best_result || best_result->target_headings_i[0] == -1) {
            // Attempt a constrained backwards-forwards soltion
            // with the minimum diameter at a low speed/no turning radius
            // don't compare costs because this result might not be feasible
            vfh_star_result_t *alt_result =
                vfh_star_update(state, easier_xy[0], easier_xy[1], 0.0, min_diam);
            if (alt_result) {
                if (best_result) {
                    vfh_star_result_destroy(best_result);
                }
                best_result = alt_result;
                best_turning_r = 0.0;
                *requires_nonturning_solution = true;
            }
        }

        // printf("VFH* update took %.3f seconds\n\n", seconds() - start);

        if (!best_result) {
            state->has_vfh_star_result = false;
            render_vfh_star(state, NULL);
            return;
        }
        if (best_result->target_headings_i[0] == -1) {
            // staying in place is a solution, but not one to act on
            state->has_vfh_star_result = false;
        } else {
            state->has_vfh_star_result = true;
        }

        state->last_turning_r = best_turning_r;

        // Mark these as out actual selected values
        // They are used as priors for the next vfh_star_update runs
        memcpy(state->chosen_directions_i, best_result->target_headings_i,
               state->goal_depth * sizeof(*state->chosen_directions_i));
        state->chosen_direction = best_result->target_heading;

        if (is_easier) {
            // try to round to current heading
            if (fabs(state->chosen_direction - state->xyt[2]) < (2 * M_PI) / state->polar_sections) {
                state->chosen_direction = state->xyt[2];
            }
        }

        // printf("Choose turn_r: %.1f direction: %d cost: %.2f\n\n", best_turning_r, best_result->target_headings_i[0], best_result->cost);

        render_vfh_star(state, best_result);
        vfh_star_result_destroy(best_result);
    }
}

void update_control(drive_to_wp_state_t *state)
{
    // make forward dist a function of velocity
    state->is_blocked_ahead = obstacle_ahead(state, &state->obstacle_ahead_slowdown);
    state->is_blocked_behind = obstacle_behind(state, &state->obstacle_behind_slowdown);
    state->is_blocked_by_sides = obstacle_by_sides(state, &state->obstacle_by_sides_slowdown);
    state->stopped_for_obstacle = false;

    waypoint_cmd_t *cmd = state->last_cmd;
    if (!cmd) {
        return;
    }

    state->control_iteration++;

    update_control_vfh(state, &state->requires_nonturning_solution);
    if (!state->has_vfh_star_result) {
        // printf("No VFH* Result!\n");
        return;
    }
    if (state->requires_nonturning_solution) {
        // use same logic, but here vfh* supplied the info
        // instead of the simple collision avoidance
        state->is_blocked_by_sides = true;
        state->obstacle_by_sides_slowdown = 0;
    }

    double min_turning_r = state->last_turning_r;
    double chosen_heading = state->chosen_direction;

    double forward_r_slowdown = min_turning_r;

    double forward_motor = 0;
    double turning_motor = 0;

    double dist = dist_to_dest(state);
    if (dist <= cmd->achievement_dist) {
        waypoint_cmd_t_destroy(state->last_cmd);
        state->last_cmd = NULL;
        return;
    }

    double heading_error = mod2pi(chosen_heading - state->xyt[2]);
    double target_velocity;
    if (!state->is_correcting_heading && fabs(heading_error) < state->heading_epsilon_rough) {
        target_velocity = state->max_velocity;
    } else {
        state->is_correcting_heading = true;
        target_velocity = state->max_velocity * forward_r_slowdown;
        if (fabs(heading_error) < state->heading_epsilon_fine) {
            state->is_correcting_heading = false;
            // basically there! reset integrators to help limit overshoot
            pid_reset_integrator(state->heading_pid);
            pid_reset_integrator(state->angular_vel_pid);
        }
    }

    // If facing in the 'wrong' direction, do not accelerate
    bool facing_wrong_direction = fabs(heading_error) >= M_PI / 2;
    if (facing_wrong_direction) {
        double limited_speed = min(fabs(state->forward_vel), fabs(target_velocity));
        if (state->forward_vel * target_velocity < 0) {
            limited_speed = 0; // current vel. is opposite desired. don't accel.
        }
        target_velocity = copysign(limited_speed, target_velocity);
    }

    if (state->is_blocked_ahead && target_velocity > 0) {
        target_velocity *= state->obstacle_ahead_slowdown;
    }
    if (state->is_blocked_behind && target_velocity < 0) {
        target_velocity *= state->obstacle_behind_slowdown;
    }

    // if close to 0, set to 0 for later logic
    if (fabs(target_velocity) < 0.01) {
        target_velocity = 0;
    }

    // is turning blocked? Then try going forwards or backwards.
    if (state->is_blocked_by_sides) {
        state->cleared_obstacle_by_sides_count = 0;

        if (state->obstacle_ahead_slowdown != 1) { //state->is_blocked_ahead) {
            state->forward_blocked_for_turn = true;
        }
        if (state->obstacle_behind_slowdown != 1) { //state->is_blocked_behind) {
            state->backward_blocked_for_turn = true;
        }

        bool velocity_valid =
            (target_velocity > 0.1 && !state->forward_blocked_for_turn) ||
            (target_velocity < -0.1 && !state->backward_blocked_for_turn);

        if (!velocity_valid) {
            if (!state->forward_blocked_for_turn &&
                    (!facing_wrong_direction || state->backward_blocked_for_turn)) {
                target_velocity = 0.1;
            } else if (!state->backward_blocked_for_turn) {
                target_velocity = -0.1;
            }
        }
    }
    if (!state->is_blocked_by_sides) {
        state->cleared_obstacle_by_sides_count++;
        if (state->cleared_obstacle_by_sides_count > 10) {
            state->forward_blocked_for_turn = false;
            state->backward_blocked_for_turn = false;
        }
    }

    pid_set_setpoint(state->velocity_pid, target_velocity);
    forward_motor = pid_compute(state->velocity_pid, state->forward_vel);

    pid_set_setpoint(state->heading_pid, 0);
    double target_ang_vel = pid_compute(state->heading_pid, -heading_error);

    if (state->is_blocked_by_sides) {
        target_ang_vel *= state->obstacle_by_sides_slowdown;
    }

    pid_set_setpoint(state->angular_vel_pid, target_ang_vel);
    turning_motor = pid_compute(state->angular_vel_pid, state->last_pose->rotation_rate[2]);

    // apply low-pass smoothing to motor commands
    uint64_t now = utime_now();
    if (state->last_motor_utime == 0) {
        state->last_motor_utime = now;
    }

    double dt = (now - state->last_motor_utime) * 1e-6;
    double alpha_factor = 2 * M_PI * dt * state->motor_low_pass_f;
    double alpha = alpha_factor / (alpha_factor + 1);
    state->last_forward = (1 - alpha) * state->last_forward + alpha * forward_motor;
    state->last_turning = (1 - alpha) * state->last_turning + alpha * turning_motor;

    state->last_motor_utime = now;

    // limits which velocities can be non-zero based on obstacles
    apply_safety_limits(state, &state->last_forward, &state->last_turning);
    if (state->last_forward == 0 && state->last_turning == 0) {
        state->stopped_for_obstacle = true;
    }

    double left_motor = state->last_forward - state->last_turning;
    double right_motor = state->last_forward + state->last_turning;

    diff_drive_t motor_cmd = {
        .utime = utime_now(),
        .left = left_motor,
        .left_enabled = true,
        .right = right_motor,
        .right_enabled = true
    };
    diff_drive_t_publish(state->lcm, "DIFF_DRIVE", &motor_cmd);

    if ((state->control_iteration % state->print_update_every) != 0) {
        return;
    }
    printf("%15s %.1f t_odo: %6.3f t_targ: %6.3f vel: %6.3f vel_targ: %6.3f avel: %6.3f avel_targ: %6.3f pid_v: %6.3f pid_t: %6.3f v_out: %6.3f t_out: %6.3f\n",
            state->is_blocked_by_sides ? "turning blocked" : "not blocked",
            min_turning_r, state->xyt[2], chosen_heading,
            state->forward_vel, state->velocity_pid->setPoint,
            state->last_pose->rotation_rate[2], state->angular_vel_pid->setPoint,
            forward_motor, turning_motor, state->last_forward, state->last_turning);
    // printf("odo: %.3f targ: %.3f p: %.3f i: %.3f d: %.3f total: %.3f out: %.3f\n",
    //        state->xyt[2], target_heading, state->heading_pid->pTerm,
    //        state->heading_pid->iTerm, state->heading_pid->dTerm,
    //        turning_motor, state->last_turning);
    // printf("%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f\n",
    //         state->xyt[2], chosen_heading, state->heading_pid->pTerm,
    //         state->heading_pid->iTerm, state->heading_pid->dTerm,
    //         turning_motor, state->last_turning,
    //         state->forward_vel, state->last_forward,
    //         min_turning_r);
}

void pid_controller_init(drive_to_wp_state_t *state, config_t *config)
{
    double vkb = config_require_double(config, "drive_to_wp.velocity_kb");
    double vkp = config_require_double(config, "drive_to_wp.velocity_kp");
    double vki = config_require_double(config, "drive_to_wp.velocity_ki");
    double vkd = config_require_double(config, "drive_to_wp.velocity_kd");
    double v_imax = config_get_double(config, "drive_to_wp.velocity_imax", 1.0);
    double v_outmax = config_get_double(config, "drive_to_wp.velocity_outmax", 1.0);
    double v_derivative_lp = config_get_double(config, "drive_to_wp.velocity_derivative_lowpass_hz", -1);

    state->velocity_pid = pid_create(vkb, vkp, vki, vkd, state->control_update_hz);
    if (v_derivative_lp != -1) {
        pid_set_derivative_filter(state->velocity_pid, v_derivative_lp, 4, 0.1);
    }
    pid_set_output_limits(state->velocity_pid, -v_outmax, v_outmax);
    pid_set_integral_limits(state->velocity_pid, -v_imax, v_imax);


    double hkb = config_require_double(config, "drive_to_wp.heading_kb");
    double hkp = config_require_double(config, "drive_to_wp.heading_kp");
    double hki = config_require_double(config, "drive_to_wp.heading_ki");
    double hkd = config_require_double(config, "drive_to_wp.heading_kd");
    double h_imax = config_get_double(config, "drive_to_wp.heading_imax", 1.0);
    double h_outmax = config_get_double(config, "drive_to_wp.heading_outmax", 1.0);
    double h_derivative_lp = config_get_double(config, "drive_to_wp.heading_derivative_lowpass_hz", -1);

    state->heading_pid = pid_create(hkb, hkp, hki, hkd, state->control_update_hz);
    if (h_derivative_lp != -1) {
        pid_set_derivative_filter(state->heading_pid, h_derivative_lp, 4, 0.1);
    }
    pid_set_output_limits(state->heading_pid, -h_outmax, h_outmax);
    pid_set_integral_limits(state->heading_pid, -h_imax, h_imax);


    double akb = config_require_double(config, "drive_to_wp.angular_vel_kb");
    double akp = config_require_double(config, "drive_to_wp.angular_vel_kp");
    double aki = config_require_double(config, "drive_to_wp.angular_vel_ki");
    double akd = config_require_double(config, "drive_to_wp.angular_vel_kd");
    double a_imax = config_get_double(config, "drive_to_wp.angular_vel_imax", 1.0);
    double a_outmax = config_get_double(config, "drive_to_wp.angular_vel_outmax", 1.0);
    double a_derivative_lp = config_get_double(config, "drive_to_wp.angular_vel_derivative_lowpass_hz", -1);

    state->angular_vel_pid = pid_create(akb, akp, aki, akd, state->control_update_hz);
    if (a_derivative_lp != -1) {
        pid_set_derivative_filter(state->angular_vel_pid, a_derivative_lp, 4, 0.1);
    }
    pid_set_output_limits(state->angular_vel_pid, -a_outmax, a_outmax);
    pid_set_integral_limits(state->angular_vel_pid, -a_imax, a_imax);

    // state->target_heading_filter = moving_filter_create(10);
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
    getopt_add_bool(gopt, '\0', "debug", 0, "Debugging mode");
    if (!getopt_parse(gopt, argc, argv, true) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(getopt_get_bool(gopt, "help") ? EXIT_SUCCESS : EXIT_FAILURE);
    }

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

    state.control_update_hz = fabs(config_require_double(config, "drive_to_wp.control_update_hz"));
    state.vfh_update_every = abs(config_require_int(config, "drive_to_wp.vfh_update_every"));
    state.print_update_every = abs(config_require_int(config, "drive_to_wp.print_update_every"));

    state.motor_low_pass_f = fabs(config_require_double(config, "drive_to_wp.motor_low_pass_freq"));
    state.max_velocity = fabs(config_require_double(config, "drive_to_wp.max_velocity"));
    state.heading_epsilon_rough = fabs(config_require_double(config, "drive_to_wp.heading_epsilon_rough"));
    state.heading_epsilon_fine = fabs(config_require_double(config, "drive_to_wp.heading_epsilon_fine"));

    state.vehicle_width = fabs(config_require_double(config, "robot.geometry.width"));
    state.vehicle_length = fabs(config_require_double(config, "robot.geometry.length"));
    state.min_side_turn_distance = fabs(config_require_double(config, "drive_to_wp.min_side_turn_distance"));
    state.min_side_back_distance = fabs(config_require_double(config, "drive_to_wp.min_side_back_distance"));
    state.min_forward_distance = fabs(config_require_double(config, "drive_to_wp.min_forward_distance"));
    state.min_forward_per_mps = fabs(config_require_double(config, "drive_to_wp.min_forward_per_mps"));

    state.cost_turning_radius = fabs(config_require_double(config, "drive_to_wp.cost_turning_radius"));

    pose_t_subscribe(state.lcm, "POSE", receive_pose, &state);
    robot_map_data_t_subscribe(state.lcm, "ROBOT_MAP_DATA", receive_robot_map_data, &state);
    waypoint_cmd_t_subscribe(state.lcm, "WAYPOINT_CMD", receive_waypoint_cmd, &state);

    initialize_vfh_star(&state, config);
    gui_init(&state);
    pid_controller_init(&state, config);

    timeutil_rest_t *timer = timeutil_rest_create();
    while (continue_running) {
        lcm_handle_async(state.lcm);

        update_control(&state);
        render_gui(&state);

        timeutil_sleep_hz(timer, state.control_update_hz);
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

    pid_destroy(state.velocity_pid);
    pid_destroy(state.heading_pid);
    // moving_filter_destroy(state.target_heading_filter);

    config_destroy(config);
    getopt_destroy(gopt);
    lcm_destroy(state.lcm);
    return 0;
}
