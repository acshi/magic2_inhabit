#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "april_graph/april_graph.h"
#include "common/config.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "common/getopt.h"
#include "common/global_map.h"
#include "common/gridmap.h"
#include "common/gridmap_util.h"
//#include "common/string_util.h"
#include "common/stype.h"
#include "common/stype_lcm.h"
#include "common/time_util.h"
#include "common/magic_util.h"
#include "hash_graph.h"
#include "lcm/lcm.h"
#include "lcmtypes/drive_command_t.h"
#include "lcmtypes/global_robot_state_t.h"
#include "lcmtypes/goal_request_t.h"
#include "lcmtypes/grid_map_t.h"
#include "lcmtypes/lcmdoubles_t.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/robot_map_data_t.h"
#include "lcmtypes/waypoint_cmd_t.h"
#include "tm_planner.h"
#include "vx/vx.h"
#include "vx/webvx.h"


// FIXME
#define FIND_NODE_THRESH 2.0  // In meters
static float grid_val_hover_rgba[4][4] = { { 0.0, 0.0, 0.0, 0.3 },
                                           { 1.0, 1.0, 0.0, 0.3 },
                                           { 1.0, 1.0, 0.0, 1.0 },
                                           { 1.0, 1.0, 0.0, 0.0 }};


static float grid_val_ref_rgba[4][4] = { { 0.0, 0.0, 0.0, 0.3 },
                                         { 0.0, 0.8, 1.0, 0.3 },
                                         { 0.0, 0.8, 1.0, 1.0 },
                                         { 0.0, 0.8, 1.0, 0.0 } };

typedef struct state state_t;
struct state
{
    lcm_t *lcm;

    config_t *config;
    vx_world_t *vw;
    webvx_t *webvx;

    april_graph_t *graph;

    double xyt[3];
    double meters_per_pixel;

    lcmdoubles_t *last_l2g;

    hash_graph_t* topo_map;
    node_t* selected_node;
    vx_console_t* vx_console;
    int64_t mouse_down_time;
    zarray_t* plan;
    double goal_xy[2];
    int plan_started;
    int start_plan; // FIXME rename

    bool vfh_star;
};

// FIXME
zarray_t *doubles_to_floats(zarray_t *zd)
{
    zarray_t *zf = zarray_create(sizeof(float[2]));
    for (int i = 0; i < zarray_size(zd); i++) {
        double *p;
        zarray_get_volatile(zd, i, &p);
        float xy[2] = { p[0], p[1] };
        zarray_add(zf, xy);
    }
    return zf;
}

// FIXME
zarray_t *floats_to_doubles(zarray_t *zf)
{
    zarray_t *zd = zarray_create(sizeof(double[2]));
    for (int i = 0; i < zarray_size(zf); i++) {
        float *p;
        zarray_get_volatile(zf, i, &p);
        double xy[2] = { p[0], p[1] };
        zarray_add(zd, xy);
    }
    return zd;
}

zarray_t *gm_to_floats(const grid_map_t *gm)
{
    assert (gm);
    zarray_t *pts = zarray_create(sizeof(float[2]));
    for (int y = 1; y + 1 < gm->height; y++) {
        for (int x = 1; x + 1 < gm->width; x++) {
            int acc = 0;
            if (0) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int yy = y + dy;
                        int xx = x + dx;
                        if (gm->data[yy*gm->width + xx] == GRID_VAL_SLAMMABLE)
                            acc ++;
                    }
                }
                if (acc >= 1) {
                    zarray_add(pts, (float[]) {
                        gm->x0 + (x + 0.5)*gm->meters_per_pixel,
                        gm->y0 + (y + 0.5)*gm->meters_per_pixel });
                }
            } else {
                if (gm->data[y*gm->width + x] == GRID_VAL_SLAMMABLE)
                    zarray_add(pts, (float[]) {
                        gm->x0 + (x + 0.5)*gm->meters_per_pixel,
                        gm->y0 + (y + 0.5)*gm->meters_per_pixel });

            }
        }
    }

    return pts;
}

int find_node(state_t *state, double xy[2], double max_distance)
{
    int a = -1;
    double bestdist2 = max_distance*max_distance;
    for (int i = 0; i < zarray_size(state->graph->nodes); i++) {
        april_graph_node_t *node;
        zarray_get(state->graph->nodes, i, &node);

        double dist2 = pow(xy[0] - node->state[0], 2) + pow(xy[1] - node->state[1], 2);
        if (dist2 < bestdist2) {
            int64_t *robot_id = ((int64_t*) april_graph_node_attr_get(node, "robot_id"));
            int64_t *ooi_id = ((int64_t*) april_graph_node_attr_get(node, "ooi_id"));
            if(!robot_id && !ooi_id) continue;

            bestdist2 = dist2;
            a = i;
        }
    }

    return a;
}

void draw_node(april_graph_node_t *node,
                          int idx,
                          float rgbas[4][4],
                          vx_buffer_t *vb)
{
    //FIXME
    //
    //
    //
    //
    vx_resource_t *tex = april_graph_node_attr_get(node, "maptex");
    grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");

    // Render observations from query node over map
    if (tex) {
        vx_object_t *vxo = vxo_image_tile(tex,
                                          rgbas[0], rgbas[1],
                                          rgbas[2], rgbas[3]);


        vx_buffer_add_back(vb,
                           vxo_matrix_xyt(node->state),
                           vxo_depth_test(0,
                                          vxo_matrix_translate(gm->x0, gm->y0, 0),
                                          vxo_matrix_scale(gm->meters_per_pixel),
                                          vxo,
                                          NULL),
                           NULL);
    }
    //else {
    //    return; // Nothing to render
    //}

    int64_t* p_robot_id = ((int64_t*) april_graph_node_attr_get(node, "robot_id"));
    int64_t* p_ooi_id = ((int64_t*) april_graph_node_attr_get(node, "ooi_id"));


    if (p_robot_id && node->type == APRIL_GRAPH_NODE_XYT_TYPE) {
        // Case 1: Robot node. Draw a triangle as usual
        // Otherwise, non-robot. Draw a circle.
        if (*p_robot_id >=2 && *p_robot_id <= 20) {
            vx_buffer_add_back(vb,
                               vxo_matrix_xyt(node->state),
                               vxo_depth_test(0,
                                              vxo_robot_solid(rgbas[GRID_VAL_SLAMMABLE]),
                                              vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                                       "<<#000000, sansserif-0.25>>%d %d", *p_robot_id, idx),
                                              NULL),
                               NULL);
        } else {
            vx_buffer_add_back(vb,
                               vxo_matrix_xyt(node->state),
                               vxo_depth_test(0,
                                              vxo_chain(
                                                        vxo_matrix_scale(0.25),
                                                        vxo_circle_solid(rgbas[GRID_VAL_SLAMMABLE]),
                                                        NULL),
                                              vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                                       "<<#000000, sansserif-0.25>>%d %d", *p_robot_id, idx),
                                              NULL),
                               NULL);
        }
    } else if (p_ooi_id && node->type == APRIL_GRAPH_NODE_XY_TYPE) {
        vx_buffer_add_back(vb,
                           vxo_matrix_xyt((double []) {node->state[0], node->state[1], 0}),
                           vxo_matrix_scale(0.2),
                           vxo_depth_test(0,
                                          vxo_circle_solid(rgbas[GRID_VAL_SLAMMABLE]),
                                          NULL),
                           NULL);
        vx_buffer_add_back(vb,
                           vxo_matrix_xyt((double []) {node->state[0], node->state[1], 0}),
                           vxo_depth_test(0,
                                          vxo_text(VXO_TEXT_ANCHOR_CENTER,
                                                   "<<#000000, sansserif-0.25>>%d\n%d", *p_ooi_id, idx),
                                          NULL),
                           NULL);
    }
}

void redraw_selected_node(state_t *state) {
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "selected_node");
    if (state->selected_node != NULL) {
        april_graph_node_t *ag_node;
        zarray_get(state->graph->nodes, state->selected_node->id, &ag_node);
        draw_node(ag_node, state->selected_node->id, grid_val_ref_rgba, vb);
    }
    vx_buffer_swap(vb);
}

void draw_topo_node(void* args, node_t* node) {
    state_t* state = ((void**) args)[0];
    vx_buffer_t* vb = ((void**) args)[1];
    april_graph_node_t *ag_node;
    zarray_get(state->graph->nodes, node->id, &ag_node);
    draw_node(ag_node, node->id, /* FIXME */ grid_val_hover_rgba, vb);
}

void draw_topo_edge(void* args, node_t* n1, node_t* n2) {
    state_t* state = ((void**) args)[0];
    vx_buffer_t* vb = ((void**) args)[1];
    april_graph_node_t *ag_node1;
    zarray_get(state->graph->nodes, n1->id, &ag_node1);
    april_graph_node_t *ag_node2;
    zarray_get(state->graph->nodes, n2->id, &ag_node2);
    vx_buffer_add_back(vb,
                       vxo_lines(vx_resource_make_attr_f32_copy((float[]) {ag_node1->state[0], ag_node1->state[1], ag_node2->state[0], ag_node2->state[1]}, 4, 2),
                       grid_val_hover_rgba[1],
                       1),
                       NULL);
}

void redraw_topo_map(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "topo_map");
    void** args = malloc(2*sizeof(void*));
    args[0] = state;
    args[1] = vb;
    hg_traverse_graph(state->topo_map, draw_topo_node, draw_topo_edge, args);
    vx_buffer_swap(vb);
}

void add_at_index(state_t* state, int index) {
    node_t* node = hg_add_node(state->topo_map, index);

    if (state->selected_node) {
        hg_add_edge(state->topo_map, state->selected_node, node);
    }
}

void draw_plan(state_t* state) {
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "plan");
    for (int i = 0; i < zarray_size(state->plan); i++) {
        node_t* node;
        zarray_get(state->plan, i, &node);
        april_graph_node_t* ag_node;
        zarray_get(state->graph->nodes, node->id, &ag_node);
        vx_buffer_add_back(
                vb,
                vxo_matrix_xyt((double []) {ag_node->state[0], ag_node->state[1], 0}),
                vxo_matrix_scale(0.2),
                vxo_circle_solid(vx_purple),
                NULL);

    }
    vx_buffer_swap(vb);
}

void draw_goal(state_t* state) {
    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "goal");
    vx_buffer_add_back(
            vb,
            vxo_matrix_xyt((double []) { state->goal_xy[0], state->goal_xy[1], 0 }),
            vxo_robot_line(vx_purple, 3),
            NULL);
    vx_buffer_swap(vb);
}

void set_goal(state_t* state, double x, double y) {
    state->goal_xy[0] = x;
    state->goal_xy[1] = y;
    printf("%f,%f\n", state->goal_xy[0], state->goal_xy[1]);
    state->start_plan = 1;
    state->plan_started = 0;
    draw_goal(state);
}

int on_mouse_click(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);

    double xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, xyz);

    int idx = find_node(state, xyz, FIND_NODE_THRESH);

    // Click to set a destination.
    if (ev->flags == 0) {
        vx_console_printf(state->vx_console, "click");
        set_goal(state, xyz[0], xyz[1]);
        printf("%f,%f\n", state->goal_xy[0], state->goal_xy[1]);
        state->start_plan = 1;
        state->plan_started = 0;
        draw_goal(state);
        return 1;
    }

    // Shift-click to add a node or edge.
    if (ev->flags == VX_EVENT_FLAGS_SHIFT) {
        vx_console_printf(state->vx_console, "shift-click");
        if (idx > -1) {
            add_at_index(state, idx);
            redraw_topo_map(state);
        }
        return 1;
    }

    // Ctrl-click to delete a node or edge.
    if (ev->flags == VX_EVENT_FLAGS_CTRL) {
        vx_console_printf(state->vx_console, "ctrl-click");
        // TODO
        return 1;
    }

    // Alt-click selects a node.
    if (ev->flags == VX_EVENT_FLAGS_ALT) {
        vx_console_printf(state->vx_console, "alt-click");
        if (idx > -1 && (state->selected_node == NULL || idx != state->selected_node->id)) {
            state->selected_node = hg_get_node(state->topo_map, idx);
            redraw_selected_node(state);
        }
        return 1;
    }
    return 0;
}

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    int ret = 0;
    switch (ev->type) {
//        case VX_EVENT_KEY_PRESSED:
//            ret = on_key_press(vl, ev, user);
//            break;
        case VX_EVENT_TOUCH_START:
        case VX_EVENT_MOUSE_DOWN:
            state->mouse_down_time = utime_now();
            break;
        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_TOUCH_END:
            if (utime_now() - state->mouse_down_time < 250*1000) {
                ret = on_mouse_click(vl, ev, user);
            }
            break;
//        case VX_EVENT_MOUSE_MOVED:
//            ret = on_mouse_move(vl, ev, user);
//            break;
        default:
            break;
    }

    return ret;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_background_rgba(vl, vx_gray, 0);

    vx_layer_set_world(vl, state->vw);

    vx_layer_add_event_handler(vl, on_event, 0, state);
    vx_console_setup_layer(state->vx_console, vl);

    int i = 0;
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "geoimage"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "graph"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "prior"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "topo_map"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "selected_node"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "plan"), i++);
    vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "cur_pose"), i++);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    printf("ON DESTROY CANVAS\n");
}

void publish_stop_command(state_t* state) {
    if (state->vfh_star) {
        waypoint_cmd_t cmd = {
            .utime = utime_now(),
            .xyt = {NAN, NAN, NAN},
            .achievement_dist = 0.3
        };
        waypoint_cmd_t_publish(state->lcm, "WAYPOINT_CMD", &cmd);
    } else {
        drive_command_t msg = {
            .stop = true
        };
        drive_command_t_publish(state->lcm, "DRIVE_COMMAND", &msg);
    }
}

void publish_drive_command(state_t* state, double* cur_xyt, double* target_xy, double* local_xyt) {
    if (state->vfh_star) {
        double delta_to_target[3];
        doubles_xyt_inv_mul(cur_xyt, target_xy, delta_to_target);
        double local_target_xyt[3];
        doubles_xyt_mul(local_xyt, delta_to_target, local_target_xyt);
        waypoint_cmd_t cmd = {
            .utime = utime_now(),
            .xyt = {local_target_xyt[0], local_target_xyt[1], NAN},
            .achievement_dist = 0.3
        };
        waypoint_cmd_t_publish(state->lcm, "WAYPOINT_CMD", &cmd);
    } else {
        double st = sin(cur_xyt[2]);
        double ct = cos(cur_xyt[2]);
        double d1 = (target_xy[0] - cur_xyt[0])*ct + (target_xy[1] - cur_xyt[1])*st;
        double d2 = -(target_xy[0] - cur_xyt[0])*st + (target_xy[1] - cur_xyt[1])*ct;
        double t_delta = atan2(d2, d1);
        printf("cur_xyt: %.2f %.2f %.2f\n", cur_xyt[0], cur_xyt[1], cur_xyt[2]);
        printf("target_xy: %.2f %.2f\n", target_xy[0], target_xy[1]);
        printf("t_delta: %.2f\n", t_delta*180.0/3.14159);
        drive_command_t msg = {
            .stop = false,
            .target_yaw = local_xyt[2] + t_delta
        };
        drive_command_t_publish(state->lcm, "DRIVE_COMMAND", &msg);
    }
}

void on_pose(const lcm_recv_buf_t *rbuf,
             const char *channel, const pose_t *pose, void *userdata)
{
    state_t *state = userdata;

    if (state->last_l2g == NULL) {
        return;
    }
    // TODO maybe dont do all this work on every single pose?
    double local_xyt[3];
    doubles_quat_xyz_to_xyt(pose->orientation,
                            pose->pos,
                            local_xyt);
    double cur_xyt[3];
    doubles_xyt_mul(state->last_l2g->data, local_xyt, cur_xyt);

    vx_buffer_t* vb = vx_world_get_buffer(state->vw, "cur_pose");
    vx_buffer_add_back(
            vb,
            vxo_matrix_xyt(cur_xyt),
            vxo_robot_solid(vx_blue),
            NULL);
    vx_buffer_swap(vb);

    if (state->start_plan) {
        state->plan = tm_plan_path(state->topo_map, state->graph->nodes, cur_xyt, state->goal_xy);
        state->start_plan = 0;
        draw_plan(state);
    }

    if (state->plan) {
        double target_xy[3];
        target_xy[2] = 0;
        if (zarray_size(state->plan)) {
            node_t* node;
            zarray_get(state->plan, 0, &node);
            april_graph_node_t* ag_node;
            zarray_get(state->graph->nodes, node->id, &ag_node);
            target_xy[0] = ag_node->state[0];
            target_xy[1] = ag_node->state[1];
        } else {
            target_xy[0] = state->goal_xy[0];
            target_xy[1] = state->goal_xy[1];
        }

        double dist = doubles_distance(target_xy, cur_xyt, 2);
        if (dist < 1) {
            if (zarray_size(state->plan)) {
                printf("Target reached.");
                zarray_remove_index(state->plan, 0, false);
                state->plan_started = 0;
            } else {
                printf("Goal reached.\n");
                state->plan = NULL;
                publish_stop_command(state);
            }
        } else if (!state->plan_started) {
            state->plan_started = 1;
            printf("Starting plan execution.\n");
            publish_drive_command(state, cur_xyt, target_xy, local_xyt);
        }
    }
}

void on_goal_request(const lcm_recv_buf_t *rbuf,
                      const char *channel,
                      const goal_request_t *msg,
                      void *user) {
    set_goal(user, msg->x, msg->y);
}

void on_l2g_scanmatch(const lcm_recv_buf_t *rbuf,
                      const char *channel,
                      const lcmdoubles_t *pos,
                      void *user) {
  state_t *state = user;

  // TODO check if we have reached end, switch policy.
  if (state->last_l2g) {
      lcmdoubles_t_destroy(state->last_l2g);
  }
  state->last_l2g = lcmdoubles_t_copy(pos);
}

grid_map_t* rmd_to_relative_gm(robot_map_data_t *msg)
{
    // the gridmap is given in robot local coordinates (at the
    // time of the message), NOT in robot-relative
    // coordinates. We will warp the gridmap so that it is in
    // robot-relative (body) coordinates.
    grid_map_t *gm_in = grid_map_t_decode_gzip(&msg->gridmap);
    assert (gm_in);

    double xytinv[3];
    doubles_xyt_inv(msg->xyt_local, xytinv);

    // compute the bounding box of the rotated gridmap.
    double miny = FLT_MAX, maxy = -FLT_MAX;
    double minx = FLT_MAX, maxx = -FLT_MAX;

    if (1) {
        // here we check every pixel
        for (int y = 0; y < gm_in->height; y++) {
            for (int x = 0; x < gm_in->width; x++) {
                uint8_t v = gm_in->data[y*gm_in->width + x];

                if (v != GRID_VAL_SLAMMABLE)
                    continue;

                double gxy[2];
                doubles_xyt_transform_xy(xytinv, (double[]) {
                    gm_in->x0 + x*gm_in->meters_per_pixel,
                    gm_in->y0 + y*gm_in->meters_per_pixel }, gxy);
                minx = fmin(minx, gxy[0]);
                maxx = fmax(maxx, gxy[0]);
                miny = fmin(miny, gxy[1]);
                maxy = fmax(maxy, gxy[1]);
            }
        }
    }

    grid_map_t *gm_out = calloc(1, sizeof(grid_map_t));
    gm_out->utime = gm_in->utime;
    gm_out->encoding = gm_in->encoding;
    gm_out->x0 = minx;
    gm_out->y0 = miny;
    gm_out->meters_per_pixel = gm_in->meters_per_pixel;
    gm_out->width = (maxx - minx) / gm_out->meters_per_pixel;
    gm_out->height = (maxy - miny) / gm_out->meters_per_pixel;
    gm_out->datalen = gm_out->width * gm_out->height;
    gm_out->data = malloc(gm_out->datalen);
    memset(gm_out->data, GRID_VAL_UNKNOWN, gm_out->datalen);

    for (int out_iy = 0; out_iy < gm_out->height; out_iy++) {

        double out_y = gm_out->y0 + gm_out->meters_per_pixel * out_iy;

        for (int out_ix = 0; out_ix < gm_out->width; out_ix++) {
            double out_x = gm_out->x0 + gm_out->meters_per_pixel * out_ix;

            double in_xy[2];
            doubles_xyt_transform_xy(msg->xyt_local, (double[]) { out_x, out_y }, in_xy);

            // where is this point in the original map?
            int in_ix = (in_xy[0] - gm_in->x0) / gm_in->meters_per_pixel;
            int in_iy = (in_xy[1] - gm_in->y0) / gm_in->meters_per_pixel;

            if (in_ix < 0 || in_ix >= gm_in->width ||
                in_iy < 0 || in_iy >= gm_in->height)
                continue;

            uint8_t v = gm_in->data[in_iy*gm_in->width + in_ix];
            gm_out->data[out_iy*gm_out->width + out_ix] = v;
        }
    }

    grid_map_t_destroy(gm_in);
    return gm_out;
}

void make_node_attributes(april_graph_node_t *node)
{
    robot_map_data_t *msg = april_graph_node_attr_get(node, "robot_map_data");
    if (!msg) return;

    if (!april_graph_node_attr_get(node, "grid_map")) {
        grid_map_t *gm_out = rmd_to_relative_gm(msg);

        // we don't need to serialize this grid map
        april_graph_node_attr_put(node, NULL, "grid_map", gm_out);
    }
}

void save_topo_node(void* args, node_t* node) {
    FILE* file = args;
    fprintf(file, "  {\n");
    fprintf(file, "    id: %d,\n", node->id);
    fprintf(file, "    edges: [\n");
    for (int i = 0; i < zarray_size(node->edges); i++) {
        node_t* node2;
        zarray_get(node->edges, i, &node2);
        fprintf(file, "      %d,\n", node2->id);
    }
    fprintf(file, "    ],\n");
    fprintf(file, "  },\n");
}

void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user) {
    state_t *state = user;

    zarray_t *toks = str_split(cmd, " ");
    if (zarray_size(toks) >= 1) {
        char *t0;
        zarray_get(toks, 0, &t0);

        // API CONS save-graph saves the current gridmap/robot pose graph
        if (!strcmp(t0, "save-graph")) {
            if (zarray_size(toks) != 2) {
                vx_console_printf(vc, "save-graph requires a file name argument.");
            } else {
                char *t1;
                zarray_get(toks, 1, &t1);
                FILE* file = fopen(t1, "w");
                fprintf(file, "[\n");
                hg_traverse_graph(state->topo_map, save_topo_node, NULL, file);
                fprintf(file, "]\n");
                fclose(file);
            }
        }
    }
    zarray_vmap(toks, free);
    zarray_destroy(toks);
}

zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user) {
    zarray_t *commands = zarray_create(sizeof(char*));
    char *cmds[] = { "save-graph ",
                     NULL };

    for (int i = 0; cmds[i] != NULL; i++) {
        if (!strncmp(cmds[i], cmd, strlen(cmd))) {
            char *s = strdup(cmds[i]);
            zarray_add(commands, &s);
        }
    }

    return commands;
}

int main(int argc, char *argv[])
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    stype_register_basic_types();
    april_graph_stype_init();
    stype_register(STYPE_FROM_LCM("grid_map_t", grid_map_t));
    stype_register(STYPE_FROM_LCM("robot_map_data_t", robot_map_data_t));
    stype_register(STYPE_FROM_LCM("global_robot_state_t", global_robot_state_t));

    state_t *state = calloc(1, sizeof(state_t));

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_bool(gopt, 'v', "vfh-star", 0, "Use vfh-star for the local planner.");
    getopt_add_string(gopt, 'c', "config", magic_util_config("robot-concat.config"), "Config file");
    getopt_add_string(gopt, '\0', "map-channel", "ROBOT_MAP_DATA.*", "LCM map data channel");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    state->meters_per_pixel = 0.05;
    state->vfh_star = getopt_get_bool(gopt, "vfh-star");
    state->config = config_create_path(getopt_get_string(gopt, "config"));
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(8205, NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    state->vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                          on_console_command,
                                          on_console_tab,
                                          state);
    state->topo_map = hg_create();
    printf("Navigate is on port %d\n", 8205);


    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }

    // load the graph and transform it
    if (1) {
        state->graph = april_graph_create_from_file(str_expand_envs(config_require_string(state->config, "navigate.graph")));
        if (state->graph == NULL) {
            fprintf(stderr, "April Graph not found\n");
            exit(1);
        }

        april_graph_t *g = state->graph;

        double graph_xyt[3];
        config_require_doubles_len(state->config, "navigate.graph_xyt", graph_xyt, 3);

	global_map_t *compositor = NULL;
        for (int i = 0; i < zarray_size(g->nodes); i++) {
            april_graph_node_t *node;
            zarray_get(g->nodes, i, &node);

            double xyt[3];
            doubles_xyt_mul(graph_xyt, node->state, xyt);
            memcpy(node->state, xyt, 3 * sizeof(double));

            make_node_attributes(node);

            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");
            grid_map_t *gmz = april_graph_node_attr_get(node, "grid_map_zip");
            if (gmz && !gm) {
                gm = grid_map_t_decode_gzip(gmz);
                april_graph_node_attr_put(node, NULL, "grid_map", gm);
            }
	    if (!compositor) {
	      compositor = global_map_create(gm->meters_per_pixel);
	    }
	    global_map_add_gridmap(compositor, gm, node->state);
        }
	//const grid_map_t* gm = global_map_get_map(compositor);
    }

    // plot the graph
    if (1) {
        april_graph_t *g = state->graph;

        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "graph");

        for (int i = 0; i < zarray_size(g->nodes); i++) {
            april_graph_node_t *node;
            zarray_get(g->nodes, i, &node);

            grid_map_t *gm = april_graph_node_attr_get(node, "grid_map");

            grid_map_t *gmz = april_graph_node_attr_get(node, "grid_map_zip");
            if (gmz && !gm) {
                gm = grid_map_t_decode_gzip(gmz);
                april_graph_node_attr_put(node, NULL, "grid_map", gm);
            }

            if (gm == NULL) {
                continue;
            }

            zarray_t *points = gm_to_floats(gm);  // Want relative grid map

            vx_resource_t *points_resc = vx_resource_make_attr_f32_copy((float*) points->data, 2*zarray_size(points), 2);

            vx_buffer_add_back(vb,
                               vxo_depth_test(0,
                                              vxo_matrix_xyt(node->state),
                                              vxo_robot_solid((float[]) { 0, .5, .5, 1 }),
                                              vxo_points(points_resc, (float[]) { 0, .5, .5, 1 }, 1),
                                              NULL),
                               NULL);


        }

        vx_buffer_swap(vb);

    }

    // Load and draw the topo map.
    if (1) {
        char *filename = str_expand_envs(config_require_string(state->config, "navigate.topo_map"));
        FILE* file = fopen(filename, "r");
        if (file == NULL) {
            fprintf(stderr, "Topo map file not found\n");
        } else {
            fscanf(file, " [");
            int id;
            while(fscanf(file, " { id: %d,", &id)) {
                node_t* n1 = hg_add_node(state->topo_map, id);
                fscanf(file, " edges: [");
                int edge;
                while(fscanf(file," %d,", &edge)) {
                    node_t* n2 = hg_add_node(state->topo_map, edge);
                    hg_add_edge(state->topo_map, n1, n2);
                }
                fscanf(file, " ], },");
            }
            fclose(file);
            redraw_topo_map(state);
        }
    }

    pose_t_subscribe(state->lcm, "POSE", on_pose, state);
    lcmdoubles_t_subscribe(state->lcm, "L2G_SCANMATCH", on_l2g_scanmatch, state);
    goal_request_t_subscribe(state->lcm, "GOAL_REQUEST", on_goal_request, state);

    while (1) {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return 0;
}
