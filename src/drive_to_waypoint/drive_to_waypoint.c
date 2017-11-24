#include "drive_to_wp_state.h"
#include "gui.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
#include "velodyne_to_map2/gridmap2.h"

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
    if (state->last_grid_map) {
        grid_map_t_destroy(state->last_grid_map);
    }
    state->last_grid_map = gridmap_decode_and_copy(&msg->gridmap);
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

double constrain(double val, double min_val, double max_val)
{
    if (val < min_val) {
        return min_val;
    } else if (val > max_val) {
        return max_val;
    }
    return val;
}

static void rasterize_poly_line(int *buff_x0, int *buff_x1, int startX, int startY, int endX, int endY) {
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

bool obstacle_ahead(drive_to_wp_state_t *state, double forward_dist, double left_dist, double right_dist) {
    grid_map_t *gm = state->last_grid_map;
    if (!gm) {
        return true;
    }

    double cos_theta = cos(state->xyt[2]);
    double sin_theta = sin(state->xyt[2]);

    double xs[4];
    double ys[4];

    xs[0] = state->xyt[0] - left_dist * sin_theta;
    ys[0] = state->xyt[1] - left_dist * cos_theta;

    xs[1] = state->xyt[0] - left_dist * sin_theta + forward_dist * cos_theta;
    ys[1] = state->xyt[1] - left_dist * cos_theta + forward_dist * sin_theta;

    xs[2] = state->xyt[0] + right_dist * sin_theta + forward_dist * cos_theta;
    ys[2] = state->xyt[1] + right_dist * cos_theta + forward_dist * sin_theta;

    xs[3] = state->xyt[0] + right_dist * sin_theta;
    ys[3] = state->xyt[1] + right_dist * cos_theta;

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

void update_control(drive_to_wp_state_t *state)
{
    state->stopped_for_obstacle = obstacle_ahead(state, OBS_FORWARD_DIST, OBS_SIDE_DIST, OBS_SIDE_DIST);
    if (state->stopped_for_obstacle) {
        diff_drive_t motor_cmd = {
            .utime = utime_now(),
            .left = 0,
            .left_enabled = false,
            .right = 0,
            .right_enabled = false
        };
        diff_drive_t_publish(state->lcm, "DIFF_DRIVE", &motor_cmd);
        return;
    }

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

    gui_init(state);

    while (1) {
        lcm_handle_async(state->lcm);

        update_control(state);
        render_gui(state);

        nanosleep(&(struct timespec){0, (CONTROL_UPDATE_MS * 1e6)}, NULL);
    }

    lcm_destroy(state->lcm);
    free(state);

    return 0;
}
