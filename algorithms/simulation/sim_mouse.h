#ifndef SIM_MOUSE_H
#define SIM_MOUSE_H

#include "sim_hal.h"
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_mouse.h  —  Continuous physical state of the simulated mouse
//
// The mouse exists in continuous (x_mm, y_mm, heading_deg) space.
// Cell-to-cell movement: receives a target cell center, drives toward it
// at fixed velocity, stops exactly on arrival. No inertia, no overshoot.
//
// Heading convention:
//   0   = North  (+y direction)
//   90  = East   (+x direction)
//   180 = South  (-y direction)
//   270 = West   (-x direction)
// ─────────────────────────────────────────────────────────────────────────────

// Default travel speed during maze-solving mode (mm/s)
#define SIM_MOUSE_SPEED_MMS 1200.0f

// How close to target counts as "arrived" (mm)
#define SIM_ARRIVAL_THRESHOLD_MM 1.0f

typedef struct {
    float x_mm;
    float y_mm;
    float heading_deg;
    float left_vel_mms;
    float right_vel_mms;
    bool  at_cell_center;
} SimMouseState;

// Initialize mouse at cell (0,0) center, heading North
void sim_mouse_init(void);

// Advance mouse position by dt seconds using current velocity
void sim_mouse_update(float dt_s);

// Command mouse to move to target cell center. Sets velocity and heading.
// Returns immediately — check sim_mouse_arrived() each tick.
void sim_mouse_move_to_cell(uint8_t target_row, uint8_t target_col);

// Returns true when mouse has reached current target cell center
bool sim_mouse_arrived(void);

// Direct velocity set — called by hal_set_motors
void sim_mouse_set_velocity(float left_mms, float right_mms);

// Read-only access to current state — called by hal_get_sensors
const SimMouseState *sim_mouse_get_state(void);

// Convert cell coordinates to continuous space (cell center)
float sim_cell_to_x(uint8_t col);
float sim_cell_to_y(uint8_t row);

// Convert continuous position to cell coordinates
uint8_t sim_x_to_col(float x_mm);
uint8_t sim_y_to_row(float y_mm);

#endif // SIM_MOUSE_H