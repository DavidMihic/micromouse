#include <stdio.h>
#include "sim_loop.h"
#include "sim_hal.h"
#include "sim_mouse.h"

// ─────────────────────────────────────────────────────────────────────────────
// sim_main.c  —  Simulation entry point
//
// This replaces main.c for the micromouse_sim build target.
// The original main.c and all of src/ are completely untouched.
//
// Build:  cmake --build . --target micromouse_sim
// Run:    ./micromouse_sim
// ─────────────────────────────────────────────────────────────────────────────

int main(void) {
    printf("=== Micromouse Simulator ===\n");
    printf("Cell size: %.0f mm  |  Maze: %dx%d\n", CELL_SIZE_MM, 16, 16);
    printf("Speed: %.0f mm/s  |  Timestep: %d ms\n\n",
           SIM_MOUSE_SPEED_MMS, SIM_TIMESTEP_MS);

    sim_loop_init();
    sim_loop_run();

    return 0;
}