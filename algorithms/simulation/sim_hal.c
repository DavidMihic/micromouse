#include "sim_hal.h"
#include "sim_mouse.h"
#include "../include/maze.h"
#include "../include/types.h"
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ─────────────────────────────────────────────────────────────────────────────
// sim_hal.c  —  Simulated HAL implementation
//
// Perfect sensors: raycasts against the ground-truth maze, returns exact
// distances. No noise, no dropout, no angular error.
//
// Perfect motors: velocity commands are applied instantaneously to sim_mouse.
// No acceleration, no slip, no encoder error.
//
// To add noise later: only this file changes. Nothing else in the project
// needs to know.
// ─────────────────────────────────────────────────────────────────────────────

// Maximum raycast distance — beyond this we report 0 (no obstacle)
#define RAY_MAX_MM 300.0f
#define RAY_STEP_MM 1.0f

// Sensor mounting angles relative to mouse heading (degrees)
// Positive = clockwise from forward
static const float sensor_angles[SENSOR_COUNT] = {
    -45.0f,   // SENSOR_FRONT_LEFT
      0.0f,   // SENSOR_FRONT
     45.0f,   // SENSOR_FRONT_RIGHT
    -90.0f,   // SENSOR_LEFT
     90.0f,   // SENSOR_RIGHT
    180.0f,   // SENSOR_BACK
};

static uint32_t sim_time_ms = 0;

// ── Raycast ──────────────────────────────────────────────────────────────────
// Cast a ray from (x, y) at angle_deg. Return distance to nearest wall or 0.

static float raycast(float x_mm, float y_mm, float angle_deg) {
    float rad = angle_deg * (float)M_PI / 180.0f;
    float dx  = sinf(rad);   // North=0 → sin gives East component
    float dy  = cosf(rad);   // North=0 → cos gives North component

    for (float dist = 0.0f; dist < RAY_MAX_MM; dist += RAY_STEP_MM) {
        float px = x_mm + dx * dist;
        float py = y_mm + dy * dist;

        // Out of maze bounds — treat as wall
        if (px < 0 || py < 0 ||
            px >= CELL_SIZE_MM * MAZE_SIZE ||
            py >= CELL_SIZE_MM * MAZE_SIZE) {
            return dist;
        }

        uint8_t col = (uint8_t)(px / CELL_SIZE_MM);
        uint8_t row = (uint8_t)(py / CELL_SIZE_MM);

        // Check if previous step was in a different cell — wall crossing
        if (dist > 0.0f) {
            float ppx = x_mm + dx * (dist - RAY_STEP_MM);
            float ppy = y_mm + dy * (dist - RAY_STEP_MM);
            uint8_t pcol = (uint8_t)(ppx / CELL_SIZE_MM);
            uint8_t prow = (uint8_t)(ppy / CELL_SIZE_MM);

            if (row != prow || col != pcol) {
                // Determine crossing direction and check for wall
                if (row > prow && maze_has_wall(prow, pcol, DIR_NORTH)) return dist;
                if (row < prow && maze_has_wall(prow, pcol, DIR_SOUTH)) return dist;
                if (col > pcol && maze_has_wall(prow, pcol, DIR_EAST))  return dist;
                if (col < pcol && maze_has_wall(prow, pcol, DIR_WEST))  return dist;
            }
        }
    }

    return 0.0f; // No obstacle within range
}

// ── HAL API ──────────────────────────────────────────────────────────────────

void hal_init(void) {
    sim_time_ms = 0;
}

void hal_shutdown(void) {
    hal_set_motors(0.0f, 0.0f);
}

void hal_get_sensors(SensorReading *out) {
    if (!out) return;

    const SimMouseState *mouse = sim_mouse_get_state();

    out->x_mm        = mouse->x_mm;
    out->y_mm        = mouse->y_mm;
    out->heading_deg = mouse->heading_deg;
    out->timestamp_ms = sim_time_ms;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        float abs_angle = mouse->heading_deg + sensor_angles[i];
        out->distance_mm[i] = raycast(mouse->x_mm, mouse->y_mm, abs_angle);
    }
}

void hal_set_motors(float left_mms, float right_mms) {
    sim_mouse_set_velocity(left_mms, right_mms);
}

// Called by sim_loop each tick to advance internal sim clock
void hal_tick(uint32_t dt_ms) {
    sim_time_ms += dt_ms;
}