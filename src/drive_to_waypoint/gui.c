#include "gui.h"
#include "common/http_advertiser.h"

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    drive_to_wp_state_t *state = impl;

    vx_canvas_set_title(vc, "Drive to Waypoint");
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_enable_camera_controls(vl, 0); // Disable camera controls
    vx_layer_set_world(vl, state->vw);
    //vx_layer_add_event_handler(vl, on_event, -100, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
}

void draw_text(vx_buffer_t *vb, double x, double y, double theta, const char *fmt, ...)
{
    // va_list processing borrowed from vxo_text
    va_list va;
    va_start(va, fmt);
    int len = vsnprintf(NULL, 0, fmt, va);
    char text[len + 1];
    va_end(va);

    va_start(va, fmt);
    vsnprintf(text, len + 1, fmt, va);
    va_end(va);

    vx_buffer_add_back(vb, vxo_chain(
                      vxo_matrix_translate(x, y, 0.1),
                      vxo_matrix_scale(0.25),
                      vxo_matrix_rotatez(theta - M_PI/2),
                      vxo_text(VXO_TEXT_ANCHOR_TOP_LEFT, "<<monospaced-bold-1,left,#8888ffff>>%s", text),
                      NULL),
                  NULL);
}

void render_robot(drive_to_wp_state_t *state, vx_buffer_t *vb) {
    vx_buffer_add_back(vb, vxo_chain(vxo_matrix_translate(state->xyt[0], state->xyt[1], 0.1),
                                        vxo_matrix_rotatez(state->xyt[2]),
                                        vxo_robot_solid(state->stopped_for_obstacle ? vx_red : vx_white),
                                        NULL),
                            NULL);
}

void render_goal(drive_to_wp_state_t *state, vx_buffer_t *vb) {
    if (!state->last_cmd) {
        return;
    }
    vx_buffer_add_back(vb, vxo_chain(vxo_matrix_translate(state->last_cmd->xyt[0], state->last_cmd->xyt[1], 0.1),
                                        vxo_matrix_scale(0.05),
                                        vxo_circle_solid(vx_green),
                                        NULL),
                            NULL);
}

void render_obs_rect(drive_to_wp_state_t *state, vx_buffer_t *vb) {
    float rect_color[] = {0.85f, 0.85f, 0, 1};

    for (double x = 0; x <= OBS_FORWARD_DIST; x += OBS_FORWARD_DIST) {
        for (double y = -OBS_SIDE_DIST; y <= OBS_SIDE_DIST; y += 2 * OBS_SIDE_DIST) {
            vx_buffer_add_back(vb, vxo_chain(vxo_matrix_translate(state->xyt[0], state->xyt[1], 0.1),
                                                vxo_matrix_rotatez(state->xyt[2]),
                                                vxo_matrix_translate(x, y, 0.1),
                                                vxo_matrix_scale(0.05),
                                                vxo_circle_solid(rect_color),
                                                NULL),
                                    NULL);
        }
    }
}

void render_gridmap(drive_to_wp_state_t *state)
{
    if(!state->last_grid_map) {
        return;
    }

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "map");

    const grid_map_t *gm = state->last_grid_map;

    float traversable_color[] = {0.3f, 0.3f, 0.3f, 1};
    float obs_color[] = {1, 0, 0, 1};
    float slam_color[] = {0, 1, 1, 1};
    float *unknown_color = vx_nada;//[] = {0, 0, 0, 1};

    image_u8_t *im = image_u8_create(gm->width, gm->height);
    for (int y = 0; y < gm->height; y++) {
        memcpy(&im->buf[y * im->stride], &gm->data[y * gm->width], gm->width);
    }

    vx_resource_t *tex = vx_resource_make_texture_u8_copy(im, 0);
    vx_object_t *vxo = vxo_image_tile(tex,
                                      traversable_color, obs_color, slam_color, unknown_color);

    vx_buffer_add_back(vb,
                       vxo_depth_test(1,
                                      vxo_matrix_translate(gm->x0, gm->y0, 0),
                                      vxo_matrix_scale(gm->meters_per_pixel),
                                      vxo,
                                      NULL),
                       NULL);
    image_u8_destroy(im);
    vx_buffer_swap(vb);
}

void render_gui(drive_to_wp_state_t *state) {
    if (!state->vw) {
        return;
    }

    // center camera over robot such that robot is always pointing "up"
    double eye[] = {state->xyt[0], state->xyt[1], 15};
    double lookat[] = {state->xyt[0], state->xyt[1], 0};
    double up[] = {cos(state->xyt[2]), sin(state->xyt[2]), 0};
    vx_world_set_elu(state->vw, eye, lookat, up, 10);

    vx_buffer_t *vb_grid = vx_world_get_buffer(state->vw, "grid");
    vx_buffer_add_back(vb_grid, vxo_matrix_translate(0, 0, 0.1), vxo_grid(vx_white, 1.0f), NULL);
    vx_buffer_swap(vb_grid);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "robot");
    render_robot(state, vb);
    render_goal(state, vb);
    render_obs_rect(state, vb);
    vx_buffer_swap(vb);

    render_gridmap(state);

    vx_buffer_t *vb_text = vx_world_get_buffer(state->vw, "text");
    //draw_text(vb_text, 1, 0, state->xyt[2], "Hello world!");
    vx_buffer_swap(vb_text);
}

void gui_init(drive_to_wp_state_t *state)
{
    if(!state->vw) {
        state->vw = vx_world_create();
        state->webvx = webvx_create_server(GUI_PORT, NULL, "index.html");
        http_advertiser_create(state->lcm, GUI_PORT, "Drive to Waypoint GUI", "Autonomous navigation to a close waypoint");

        webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

        vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "map"), 100);
        vx_buffer_set_draw_order(vx_world_get_buffer(state->vw, "robot"), 200);
    }
}
