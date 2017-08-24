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
            break;
        }
        case VX_EVENT_MOUSE_MOVED: // HACK!
        case VX_EVENT_TOUCH_MOVE: {
            break;
        }
        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_TOUCH_END: {
            break;
        }
        case VX_EVENT_CANVAS_ECHO: {
            client->last_echo = utime_now();
            client->watchdog_timeout = 0;
            break;
        }
        case VX_EVENT_CANVAS_CHANGE: {
            // Orientation change wedges touch interface. Start fresh.
            pthread_mutex_lock(&client->mutex);
            client->finger_down = 0;
            client->finger_id = -1;
            pthread_mutex_unlock(&client->mutex);
        }
        default: {
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

        vx_canvas_echo(client->vc, nonce);
        nonce++;

        usleep(WATCHDOG_PERIOD * 1000);
    }

    return NULL;
}

void *render_thread(void *user) {
    client_t *client = user;
    state_t *state = client->state;

    for (int iter = 0; 1; iter++) {
        usleep(100e3);
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
    vx_canvas_set_title(client->vc, "Tag Follower");
    vx_layer_add_event_handler(client->vl, event_handler, 0, client);
    vx_layer_enable_camera_controls(client->vl, 0); // turn off touch camera controls
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
        lcm_handle(state->lcm);
    }
}
