#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_hal.h  —  Hardware Abstraction Layer contract
//
// This is the ONLY file the navigation layer (floodfill, planner, state)
// is allowed to call for physical interaction. On hardware, a firmware team
// replaces sim_hal.c with a real implementation. The struct and function
// signatures here must never change without coordinating both sides.
//
// Sensor layout (top-down view, mouse facing North):
//
//          [FL]  [F]  [FR]
//           \    |    /
//        [L]      [R]
//
// Six sensors total — front-left, front, front-right, left, right, back
// Distances in millimeters. 0.0f means no obstacle detected (beyond range).
// ─────────────────────────────────────────────────────────────────────────────

#define CELL_SIZE_MM     180.0f
#define MAZE_ORIGIN_X_MM  90.0f   // center of cell (0,0) in continuous space
#define MAZE_ORIGIN_Y_MM  90.0f

// Sensor indices — use these, never raw numbers
#define SENSOR_FRONT_LEFT   0
#define SENSOR_FRONT        1
#define SENSOR_FRONT_RIGHT  2
#define SENSOR_LEFT         3
#define SENSOR_RIGHT        4
#define SENSOR_BACK         5
#define SENSOR_COUNT        6

typedef struct {
    float distance_mm[SENSOR_COUNT];   // per-sensor distance readings
    float x_mm;                        // continuous x position
    float y_mm;                        // continuous y position
    float heading_deg;                 // 0=North, 90=East, 180=South, 270=West
    uint32_t timestamp_ms;             // sim time, not wall time
} SensorReading;

// Called by navigation layer to get current sensor state.
// In sim: raycasts against real maze. On hardware: reads IR + EKF pose.
void hal_get_sensors(SensorReading *out);

// Called by navigation layer to command wheel velocities.
// In sim: sets simulated mouse velocity. On hardware: feeds motor PID.
// Positive = forward. Units: mm/s per wheel.
void hal_set_motors(float left_mms, float right_mms);

// Called once at startup to initialize the HAL layer.
void hal_init(void);

// Called to shut down HAL cleanly (sim: no-op, hardware: safe motor stop).
void hal_shutdown(void);

// Advances simulation time by dt_ms
void hal_tick(uint32_t dt_ms);

#endif // SIM_HAL_H