#pragma once

#include "drive_to_wp_state.h"

#define GUI_PORT 8765

void gui_init(drive_to_wp_state_t *state);
void render_gui(drive_to_wp_state_t *state);
