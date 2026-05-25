#include "sim_mouse.h"
#include "../include/types.h"
#include <math.h>
#include <stddef.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_mouse.c  —  Mouse kinematics
//
// Cell-to-cell mode: mouse moves in a straight line toward target cell center
// at fixed speed. Heading is snapped to cardinal direction (N/E/S/W).
// Arrival is checked by distance threshold each tick.
//
// No acceleration. No rotation animation. Heading changes are instantaneous.
// This keeps the maze-solving phase clean and debuggable.
// ─────────────────────────────────────────────────────────────────────────────

static SimMouseState state;

static float target_x_mm = 0.0f;
static float target_y_mm = 0.0f;
static bool  moving      = false;

void sim_mouse_init(void) {
    state.x_mm          = sim_cell_to_x(0);
    state.y_mm          = sim_cell_to_y(0);
    state.heading_deg   = 0.0f;   // North
    state.left_vel_mms  = 0.0f;
    state.right_vel_mms = 0.0f;
    state.at_cell_center = true;

    target_x_mm = state.x_mm;
    target_y_mm = state.y_mm;
    moving = false;
}

void sim_mouse_update(float dt_s) {
    if (!moving) return;

    float dx = target_x_mm - state.x_mm;
    float dy = target_y_mm - state.y_mm;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist <= SIM_ARRIVAL_THRESHOLD_MM) {
        // Snap exactly to target — no floating point drift
        state.x_mm = target_x_mm;
        state.y_mm = target_y_mm;
        state.left_vel_mms  = 0.0f;
        state.right_vel_mms = 0.0f;
        state.at_cell_center = true;
        moving = false;
        return;
    }

    // Straight-line advance toward target at current speed
    float speed = (state.left_vel_mms + state.right_vel_mms) * 0.5f;
    float step  = speed * dt_s;

    if (step >= dist) {
        state.x_mm = target_x_mm;
        state.y_mm = target_y_mm;
        state.left_vel_mms  = 0.0f;
        state.right_vel_mms = 0.0f;
        state.at_cell_center = true;
        moving = false;
    } else {
        state.x_mm += (dx / dist) * step;
        state.y_mm += (dy / dist) * step;
        state.at_cell_center = false;
    }
}

void sim_mouse_move_to_cell(uint8_t target_row, uint8_t target_col) {
    target_x_mm = sim_cell_to_x(target_col);
    target_y_mm = sim_cell_to_y(target_row);

    // Snap heading to cardinal direction of movement
    float dx = target_x_mm - state.x_mm;
    float dy = target_y_mm - state.y_mm;

    if      (dy >  0.1f) state.heading_deg =   0.0f;  // North
    else if (dx >  0.1f) state.heading_deg =  90.0f;  // East
    else if (dy < -0.1f) state.heading_deg = 180.0f;  // South
    else if (dx < -0.1f) state.heading_deg = 270.0f;  // West

    state.left_vel_mms  = SIM_MOUSE_SPEED_MMS;
    state.right_vel_mms = SIM_MOUSE_SPEED_MMS;
    state.at_cell_center = false;
    moving = true;
}

bool sim_mouse_arrived(void) {
    return state.at_cell_center;
}

void sim_mouse_set_velocity(float left_mms, float right_mms) {
    state.left_vel_mms  = left_mms;
    state.right_vel_mms = right_mms;
    moving = (left_mms != 0.0f || right_mms != 0.0f);
}

const SimMouseState *sim_mouse_get_state(void) {
    return &state;
}

float sim_cell_to_x(uint8_t col) {
    return col * CELL_SIZE_MM + MAZE_ORIGIN_X_MM;
}

float sim_cell_to_y(uint8_t row) {
    return row * CELL_SIZE_MM + MAZE_ORIGIN_Y_MM;
}

uint8_t sim_x_to_col(float x_mm) {
    if (x_mm < 0) return 0;
    uint8_t col = (uint8_t)(x_mm / CELL_SIZE_MM);
    return col < MAZE_SIZE ? col : MAZE_SIZE - 1;
}

uint8_t sim_y_to_row(float y_mm) {
    if (y_mm < 0) return 0;
    uint8_t row = (uint8_t)(y_mm / CELL_SIZE_MM);
    return row < MAZE_SIZE ? row : MAZE_SIZE - 1;
}