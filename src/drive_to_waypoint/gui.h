#pragma once

#include "common/general_search.h"

#include "drive_to_wp_state.h"
#include "vfh_star.h"

#define GUI_PORT 8765

void gui_init(drive_to_wp_state_t *state);
void render_gui(drive_to_wp_state_t *state);

void render_vfh_star(drive_to_wp_state_t *state, vfh_star_result_t *vfh_result);
