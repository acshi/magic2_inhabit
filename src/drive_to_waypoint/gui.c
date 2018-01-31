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

    vx_buffer_add_back(vb, vxo_matrix_translate(x, y, 0.1),
                            vxo_matrix_scale(0.25),
                            vxo_matrix_rotatez(theta - M_PI/2),
                            vxo_text(VXO_TEXT_ANCHOR_TOP_LEFT, "<<monospaced-bold-1,left,#8888ffff>>%s", text),
                            NULL);
}

void render_robot(drive_to_wp_state_t *state, vx_buffer_t *vb)
{
    vx_buffer_add_back(vb, vxo_matrix_translate(state->xyt[0], state->xyt[1], 0.2),
                            vxo_matrix_rotatez(state->xyt[2]),
                            vxo_robot_solid(state->stopped_for_obstacle ? vx_red : vx_white),
                            NULL);
}

void render_goal(drive_to_wp_state_t *state, vx_buffer_t *vb)
{
    if (!state->last_cmd) {
        return;
    }
    vx_buffer_add_back(vb, vxo_matrix_translate(state->last_cmd->xyt[0], state->last_cmd->xyt[1], 0.1),
                            vxo_matrix_scale(0.05),
                            vxo_circle_solid(vx_green),
                            NULL);
}

void render_obs_rect(drive_to_wp_state_t *state, vx_buffer_t *vb)
{
    float rect_color[] = {0.85f, 0.85f, 0, 1};

    double forward_dist = max(state->min_forward_distance, state->min_forward_per_mps * state->forward_vel);
    double left_dist = state->min_side_distance + state->vehicle_width / 2;
    double right_dist = left_dist;

    for (double x = 0; x <= forward_dist; x += forward_dist) {
        for (double y = -left_dist; y <= right_dist; y += left_dist + right_dist) {
            vx_buffer_add_back(vb, vxo_matrix_translate(state->xyt[0], state->xyt[1], 0.1),
                                    vxo_matrix_rotatez(state->xyt[2]),
                                    vxo_matrix_translate(x, y, 0.1),
                                    vxo_matrix_scale(0.05),
                                    vxo_circle_solid(rect_color),
                                    NULL);
        }
    }
}

vx_object_t *gui_image_gridmap(vx_resource_t *tex, float rgba0[4], float rgba1[4], float rgba2[4], float rgba3[4])
{
    float w = vx_resource_texture_get_width(tex), h = vx_resource_texture_get_height(tex);
    float tw = 1.0f * vx_resource_texture_get_bpp(tex) * w / vx_resource_texture_get_stride(tex), th = 1.0f;
    vx_resource_t *position_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  w, 0,  0, h,  w, h }, 8, 2);
    vx_resource_t *texcoord_resource = vx_resource_make_attr_f32_copy((float[]) { 0, 0,  tw, 0,  0, th, tw, th }, 8, 2);

    vx_lock();

    static vx_resource_t *program_resource = NULL;

    if (program_resource == NULL) {

        char vertex_shader_src[] =
            "attribute vec2 aposition; \n"  \
            "attribute vec2 atexcoord; \n"  \
            "varying vec2 vtexcoord; \n"   \
            "uniform mat4 VX_P;\n"         \
            "uniform mat4 VX_V;\n"         \
            "uniform mat4 VX_M;\n"         \
            "void main(void) {\n"          \
            "  vtexcoord = atexcoord; \n " \
            "  gl_Position = VX_P * VX_V * VX_M * vec4(aposition, 0, 1.0);\n" \
            "}";

        // The color map from GRID_FLAG vals:
        // GRID_FLAG_NONE (0) : (0, 0, 0, 0)
        // GRID_FLAG_TRAVERSABLE (1): rgba0
        // GRID_FLAG_OBSTACLE (2): rgba1
        // GRID_FLAG_SLAMMABLE (4): rgba2
        // GRID_FLAG_TRAVERSABLE | GRID_FLAG_SLAMMABLE (5): rgba3
        // GRID_FLAG_OBSTACLE | GRID_FLAG_SLAMMABLE (6): rgba2

        char fragment_shader_src[] =
            "precision mediump float; \n"   \
            "varying vec2 vtexcoord; \n"         \
            "uniform sampler2D texture; \n" \
            "uniform vec4 rgba0, rgba1, rgba2, rgba3; \n" \
            "void main(void) {\n"           \
            "  vec4 c = texture2D(texture, vtexcoord);\n" \
            "  if (c.r > 5.5 / 255.0) gl_FragColor = rgba2;\n" \
            "  else if (c.r > 4.5 / 255.0) gl_FragColor = rgba3;\n" \
            "  else if (c.r > 3.5 / 255.0) gl_FragColor = rgba2;\n" \
            "  else if (c.r > 1.5 / 255.0) gl_FragColor = rgba1;\n" \
            "  else if (c.r > 0.5 / 255.0) gl_FragColor = rgba0;\n"
            "  else gl_FragColor = vec4(0.0, 0.0, 0.0, 0);\n" \
            "}\n";

        program_resource = vx_resource_make_program(vertex_shader_src, fragment_shader_src);
        program_resource->incref(program_resource); // make immortal
    }

    vx_unlock();

    return vxo_generic_create(program_resource,
                              (struct vxo_generic_uniformf[]) {
                                  { .name="rgba0", .nrows = 4, .ncols = 1, .data = rgba0 },
                                  { .name="rgba1", .nrows = 4, .ncols = 1, .data = rgba1 },
                                  { .name="rgba2", .nrows = 4, .ncols = 1, .data = rgba2 },
                                  { .name="rgba3", .nrows = 4, .ncols = 1, .data = rgba3 },
                                  { .name=NULL } },
                              (struct vxo_generic_attribute[]) {
                                  { .name="aposition", .resource=position_resource },
                                  { .name="atexcoord", .resource=texcoord_resource },
                                  { .name=NULL } },
                              (struct vxo_generic_texture[]) {
                                  { .name="texture", .resource = tex },
                                  { .name = NULL } },
                              (struct vxo_generic_draw[]) {
                                  { .command = VX_GL_TRIANGLE_STRIP, .first = 0, .count = 4 },
                                  { .count = 0 }, });
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
    float traversable_slam_color[] = {1, 1, 1, 1};

    image_u8_t *im = image_u8_create(gm->width, gm->height);
    for (int y = 0; y < gm->height; y++) {
        memcpy(&im->buf[y * im->stride], &gm->data[y * gm->width], gm->width);
    }

    vx_resource_t *tex = vx_resource_make_texture_u8_copy(im, 0);
    vx_object_t *vxo = gui_image_gridmap(tex,
                                         traversable_color, obs_color,
                                         slam_color, traversable_slam_color);

    vx_buffer_add_back(vb,
                          vxo_matrix_translate(gm->x0, gm->y0, 0),
                          vxo_matrix_scale(gm->meters_per_pixel),
                          vxo,
                          NULL);
    image_u8_destroy(im);
    vx_buffer_swap(vb);
}

void render_gui(drive_to_wp_state_t *state)
{
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

void render_vfh_star(drive_to_wp_state_t *state, gen_search_node_t *result)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "vfh_star");
    gen_search_node_t *parent = result->parent;
    while (parent) {
        vfh_plus_t *vfh = (vfh_plus_t*)result->state;
        vfh_plus_t *prior_vfh = (vfh_plus_t*)parent->state;

        vx_buffer_add_back(vb, vxo_matrix_translate(vfh->xyt[0], vfh->xyt[1], 0.2),
                                vxo_matrix_rotatez(vfh->xyt[2]),
                                vxo_robot_solid(vx_green),
                                NULL);

        float line[4] = {
            (float)vfh->xyt[0],
            (float)vfh->xyt[1],
            (float)prior_vfh->xyt[0],
            (float)prior_vfh->xyt[1],
        };
        vx_resource_t *vr = vx_resource_make_attr_f32_copy(line, 4, 2);
        vx_buffer_add_back(vb, vxo_lines(vr, vx_white, 1), NULL);

        result = parent;
        parent = result->parent;
    }
    vx_buffer_swap(vb);
}

void gui_init(drive_to_wp_state_t *state)
{
    if(!state->vw) {
        state->vw = vx_world_create();
        state->webvx = webvx_create_server(GUI_PORT, NULL, "index.html");
        http_advertiser_create(state->lcm, GUI_PORT, "Drive to Waypoint GUI", "Autonomous navigation to a close waypoint");

        webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    }
}
