#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "vx/vx.h"
#include "vx/webvx.h"
#include "common/time_util.h"
#include "lcm/lcm.h"

#define WATCHDOG_PERIOD    50 // ms
#define WATCHDOG_TIMEOUT  500 // ms
#define DRIVE_PERIOD       40 // ms
#define JOYPAD_BOUNDS     200 // css pixels
#define JOYSTICK_SIZE     100 // css pixels

#define MODE_JOYPAD 0
#define MODE_AUTON 1
#define DISABLE_BOX_X 750
#define DISABLE_BOX_Y 300

uint32_t next_id = 0;

typedef struct client client_t;
typedef struct state state_t;

struct client
{
    state_t *state;

    uint32_t id;

    vx_world_t *vw;
    vx_canvas_t *vc;
    vx_layer_t  *vl;
    pthread_t watchdog;
    uint64_t last_echo;
    uint8_t watchdog_timeout;
    pthread_t render_thread;

    // Joypad screen coordinates
    uint8_t finger_down;
    int finger_id;
    float current_x;
    float current_y;
    float initial_x;
    float initial_y;

    // Override interface
    uint8_t override;
    int second_finger_id;

    // Signal for threads
    uint8_t close;

    pthread_mutex_t mutex;
};

struct state
{
    lcm_t *lcm;

    webvx_t *webvx;
    zhash_t *clients;

    client_t *control_client;

    // Signal to stop publishing
    uint8_t mode;

    pthread_mutex_t mutex;
};

int event_handler(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    client_t *client = user;
    state_t *state = client->state;

    switch (ev->type) {

        case VX_EVENT_MOUSE_DOWN: // HACK!
        case VX_EVENT_TOUCH_START: {
            //printf("Touch %d start at %f, %f\n", ev->u.touch.id, ev->u.touch.x, ev->u.touch.y);
            // second finger initiates override
            if (client->finger_down && !client->override) {
                pthread_mutex_lock(&client->mutex);
                client->override = 1;
                client->second_finger_id = ev->u.touch.id;
                pthread_mutex_unlock(&client->mutex);

                float override_red[] = {0.9, 0.1, 0.1, 1.0};
                vx_layer_set_background_rgba(client->vl, override_red);
            }

            if (!client->finger_down) {
                // Screen rotation sometimes breaks this assertion
                //assert(client->finger_down == 0);

                pthread_mutex_lock(&client->mutex);
                client->finger_id = ev->u.touch.id;
                client->initial_x = ev->u.touch.x;;
                client->initial_y = ev->viewport[3] - ev->u.touch.y;
                client->current_x = client->initial_x;
                client->current_y = client->initial_y;

                client->finger_down = 1; // TODO: use a lock here, for now 8bit write is atomic.
                pthread_mutex_unlock(&client->mutex);

                // Grab robot control if nobody else has it.
                pthread_mutex_lock(&state->mutex);
                if(state->control_client == NULL) {
                    state->control_client = client;
                }
                pthread_mutex_unlock(&state->mutex);
            }

            pthread_mutex_lock(&state->mutex);

            if(ev->u.touch.x > ev->viewport[2] - DISABLE_BOX_X
               && ev->u.touch.x < ev->viewport[2] - 0
               && ev->u.touch.y > ev->viewport[3] - DISABLE_BOX_Y
               && ev->u.touch.y < ev->viewport[3] - 0)
            {
                state->mode = (state->mode + 1) % 2;
            }
            else
                state->mode = MODE_JOYPAD;

            pthread_mutex_unlock(&state->mutex);

            break;
        }
        case VX_EVENT_MOUSE_MOVED: // HACK!
        case VX_EVENT_TOUCH_MOVE: {
            //printf("Touch %d move at %f, %f\n", ev->u.touch.id, ev->u.touch.x, ev->u.touch.y);
            if (ev->u.touch.id == client->finger_id) {
                //assert(client->finger_down == 1);

                pthread_mutex_lock(&client->mutex);
                client->current_x = ev->u.touch.x;
                client->current_y = ev->viewport[3] - ev->u.touch.y;
                pthread_mutex_unlock(&client->mutex);
            }
            break;
        }
        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_TOUCH_END: {
            //printf("Touch %d end   at %f, %f\n", ev->u.touch.id, ev->u.touch.x, ev->u.touch.y);
            if (ev->u.touch.id == client->finger_id) {
                pthread_mutex_lock(&client->mutex);
                //assert(client->finger_down == 1);
                client->finger_down = 0; // TODO: use a lock here, for now 8bit write is atomic.
                pthread_mutex_unlock(&client->mutex);

                // Release robot control if I have it.
                pthread_mutex_lock(&state->mutex);
                if(state->control_client == client) {
                    state->control_client = NULL;
                }
                pthread_mutex_unlock(&state->mutex);
            }

            // second finger stops override
            if (client->override && ev->u.touch.id == client->second_finger_id) {
                pthread_mutex_lock(&client->mutex);
                client->override = 0;
                pthread_mutex_unlock(&client->mutex);
                float normal_gray[] = {0.2, 0.2, 0.2, 1.0};
                vx_layer_set_background_rgba(client->vl, normal_gray);
            }
            break;
        }
        case VX_EVENT_ECHO: {

            // Pet watchdog
            // XXX: Mutex?
            client->last_echo = utime_now();
            client->watchdog_timeout = 0;
            //printf("ECHO: %u %" PRIu64 ", utime: %" PRIu64 "\n", client->id, ev->u.echo.nonce, client->last_echo);

            break;
        }
        case VX_EVENT_ORIENTATION_CHANGE: {
            // Orientation change wedges touch interface. Start fresh.
            pthread_mutex_lock(&client->mutex);
            client->finger_down = 0;
            client->finger_id = -1;
            pthread_mutex_unlock(&client->mutex);
        }
        default: {
            //printf("unknown event %d\n", ev->type);

            break;
        }
    }

    return 0;
}

void *watchdog_thread(void *user) {

    client_t *client = user;
    uint64_t nonce = 0;
    uint64_t last_echo;

    printf("starting watchdog for client %u\n", client->id);

    while (1) {

        if (client->close) {
            printf("Client %u watchdog thread exiting\n", client->id);
            return NULL;
        }

        last_echo = client->last_echo;

        if ((utime_now() - last_echo) > (WATCHDOG_TIMEOUT * 1000)) {
            client->watchdog_timeout = 1;
        }

        vx_layer_echo(client->vl, nonce);
        nonce++;

        usleep(WATCHDOG_PERIOD * 1000);
    }

    return NULL;
}

void *render_thread(void *user) {

    client_t *client = user;
    state_t *state = client->state;

    for (int iter = 0; 1; iter++) {
        if (1) {

            pthread_mutex_lock(&state->mutex);
            uint8_t nclients = zhash_size(state->clients);
            uint8_t mode = state->mode;
            pthread_mutex_unlock(&state->mutex);

            pthread_mutex_lock(&client->mutex);
            uint8_t finger_down = client->finger_down;
            float current_x = client->current_x;
            float current_y = client->current_y;
            float initial_x = client->initial_x;
            float initial_y = client->initial_y;
            uint8_t client_close = client->close;
            pthread_mutex_unlock(&client->mutex);

            if (client->close) {
                printf("Client %u render thread exiting\n", client->id);
                return NULL;
            }

            vx_buffer_t *vb = vx_world_get_buffer(client->vw, "robot");
            float color[4] = {1.0, 1.0, 1.0, 1.0};

            // Draw Joypad boundary
            if (finger_down) {
                vx_buffer_add_back(vb, vxo_pixcoords(
                                       VXO_PIXCOORDS_BOTTOM_LEFT,
                                       VXO_PIXCOORDS_SCALE_MODE_ONE,
                                       vxo_chain(vxo_matrix_translate(initial_x, initial_y, 0),
                                                 vxo_matrix_scale(JOYPAD_BOUNDS),
                                                 vxo_circle_line(color, 2.0),
                                                 NULL),
                                       NULL),
                                   NULL);

                // calculate joypad bounds
                matd_t *I = matd_create_data(1, 3, (double[]){initial_x, initial_y, 1});
                matd_t *C = matd_create_data(1, 3, (double[]){current_x, current_y, 1});
                matd_t *X = matd_op("M-M", C, I);
                double x = current_x;
                double y = current_y;
                if (matd_vec_mag(X) > JOYPAD_BOUNDS) {
                    // scale
                    matd_t *X_norm = matd_vec_normalize(X);
                    matd_t *X_scale = matd_scale(X_norm, JOYPAD_BOUNDS);
                    matd_t *C_new = matd_op("M+M", I, X_scale);
                    x = C_new->data[0];
                    y = C_new->data[1];

                    matd_destroy(X_norm);
                    matd_destroy(X_scale);
                    matd_destroy(C_new);
                }
                matd_destroy(I);
                matd_destroy(C);
                matd_destroy(X);

                // Draw Joypad
                vx_buffer_add_back(vb, vxo_pixcoords(
                                       VXO_PIXCOORDS_BOTTOM_LEFT,
                                       VXO_PIXCOORDS_SCALE_MODE_ONE,
                                       vxo_chain(vxo_matrix_translate(x, y, 0),
                                                 vxo_matrix_scale(JOYSTICK_SIZE),
                                                 vxo_circle_solid(color),
                                                 NULL),
                                       NULL),
                                   NULL);
            }

            // Draw connection signal blinky light
            float red[] = {1.0, 0.0, 0.0, 1.0};
            float green[] = {0.0, 1.0, 0.0, 1.0};
            float *colors[] = {red, green};
            uint64_t blink_prd = 500 * 1000;
            uint64_t which_color = (utime_now() / blink_prd) % 2;
            double rpm = 30.0;
            double rad = (((utime_now() / (1000)) / (60.0 * 1000.0)) * rpm) * (2 * M_PI);
            vx_buffer_add_back(vb, vxo_pixcoords(
                                   VXO_PIXCOORDS_TOP_RIGHT,
                                   VXO_PIXCOORDS_SCALE_MODE_ONE,
                                   vxo_chain(vxo_matrix_scale(40),
                                             vxo_matrix_translate(-1.5, -1.5, 0),
                                             vxo_matrix_rotatez(rad),
                                             vxo_robot_solid(colors[1]),
                                             NULL),
                                   NULL),
                               NULL);

            vx_buffer_swap(vb);

            vb = vx_world_get_buffer(client->vw, "nrobots");
            vx_buffer_add_back(vb, vxo_pixcoords(
                                   VXO_PIXCOORDS_BOTTOM_LEFT,
                                   VXO_PIXCOORDS_SCALE_MODE_ONE,
                                   vxo_chain(vxo_matrix_scale(4),
                                             vxo_text(VXO_TEXT_ANCHOR_BOTTOM_LEFT,
                                                      "%d",
                                                      nclients),
                                             NULL),
                                   NULL),
                               NULL);
            vx_buffer_swap(vb);

            vb = vx_world_get_buffer(client->vw, "disable");
            if(mode == MODE_AUTON)
            {
                vx_buffer_add_back(vb, vxo_pixcoords(
                                       VXO_PIXCOORDS_BOTTOM_RIGHT,
                                       VXO_PIXCOORDS_SCALE_MODE_ONE,
                                       vxo_chain(vxo_matrix_scale3(DISABLE_BOX_X, DISABLE_BOX_Y, 1),
                                                 vxo_square_solid((float[]){0,1,0,1}),
                                                 NULL),
                                       vxo_chain(vxo_matrix_scale(6),
                                                 vxo_matrix_translate(-5,0,0.1),
                                                 vxo_text(VXO_TEXT_ANCHOR_BOTTOM_RIGHT,
                                                          "AUTON"),
                                                 NULL),
                                       NULL),
                                   NULL);
            }
            else
            {
                vx_buffer_add_back(vb, vxo_pixcoords(
                                       VXO_PIXCOORDS_BOTTOM_RIGHT,
                                       VXO_PIXCOORDS_SCALE_MODE_ONE,
                                       vxo_chain(vxo_matrix_scale3(DISABLE_BOX_X, DISABLE_BOX_Y, 1),
                                                 vxo_square_solid((float[]){0,0,1,1}),
                                                 NULL),
                                       vxo_chain(vxo_matrix_scale(6),
                                                 vxo_matrix_translate(-5,0,0.1),
                                                 vxo_text(VXO_TEXT_ANCHOR_BOTTOM_RIGHT,
                                                          "JOYPD"),
                                                 NULL),
                                       NULL),
                                   NULL);

            }
            vx_buffer_swap(vb);
        }

        usleep(100*1000);
    }
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    //if (next_id == 1) return;
    printf("canvas at %" PRIu64 "\n", (uint64_t)vc);

    state_t *state = impl;

    printf("ON CREATE CANVAS\n");
    client_t *client = calloc(1, sizeof(client_t));

    // set up client environment
    client->id = next_id;
    next_id++;

    client->finger_down = 0;
    client->finger_id = -1;

    client->state = state;
    client->vw = vx_world_create();
    client->vc = vc;
    char layer_name[32];
    sprintf(layer_name, "default_%u", client->id);
    client->vl = vx_canvas_get_layer(vc, layer_name);
    vx_layer_set_world(client->vl, client->vw);
    vx_layer_set_title(client->vl, "Joypad Drive");
    vx_layer_add_event_handler(client->vl, event_handler, 0, client);
    vx_layer_enable_touch_camera_controls(client->vl, 0); // turn off touch camera controls
    pthread_mutex_init(&client->mutex, NULL);

    // start threads
    pthread_mutex_lock(&client->mutex);
    client->close = 0;
    pthread_create(&client->watchdog, NULL, watchdog_thread, client);
    pthread_create(&client->render_thread, NULL, render_thread, client);

    // Add client to zhash
    zhash_put(state->clients, &client->vc, &client, NULL, NULL);
    printf("clients: %u \n", zhash_size(state->clients));

    client_t *client_test = NULL;
    zhash_get(state->clients, &client->vc, &client_test);
    assert(client_test == client);

    pthread_mutex_unlock(&client->mutex);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
    // XXX: Clean up
    client_t *client = NULL;
    zhash_get(state->clients, &vc, &client);

    pthread_mutex_unlock(&state->mutex);
    client->close = 1;
    pthread_join(client->watchdog, NULL);
    pthread_join(client->render_thread, NULL);

    free(client);

    zhash_remove(state->clients, &vc, NULL, NULL);
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));
    state->lcm = lcm_create(NULL);
    pthread_mutex_init(&state->mutex, NULL);
    state->clients = zhash_create(sizeof(vx_canvas_t*), sizeof(client_t*),
                                  zhash_ptr_hash, zhash_ptr_equals);
    state->control_client = NULL;
    state->webvx = webvx_create_server(8123, NULL, "index.html");

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    printf("CLICK ME: http://localhost:8123\n");

    while (1) {
        usleep(10000);
    }
}
