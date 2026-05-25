#include "sim_loop.h"
#include "sim_hal.h"
#include "sim_mouse.h"
#include "sim_render.h"
#include "sim_nav.h"
#include "../include/types.h"
#include <SDL2/SDL.h>
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_loop.c  —  Fixed-timestep simulation loop
//
// Coordinates: mouse physics → arrival check → navigation step → render.
// The timestep is fixed at SIM_TIMESTEP_MS. Real wall-clock time is used
// only for SDL frame pacing — the sim itself runs on sim time.
// ─────────────────────────────────────────────────────────────────────────────

static SimStatus status = {
    .phase       = SIM_PHASE_EXPLORING,
    .run_number  = 0,
    .total_steps = 0,
    .maze_solved = false
};

static bool running = false;

void sim_loop_init(void) {
    hal_init();
    sim_mouse_init();
    sim_nav_init();
    sim_render_init();

    status.phase       = SIM_PHASE_EXPLORING;
    status.run_number  = 1;
    status.total_steps = 0;
    status.maze_solved = false;

    running = true;
    printf("[sim] Initialized. Starting run %d.\n", status.run_number);
}

void sim_loop_run(void) {
    const float dt_s = SIM_TIMESTEP_MS / 1000.0f;

    while (running) {
        uint32_t frame_start = SDL_GetTicks();

        // ── Handle SDL events ─────────────────────────────────────────────
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = false;
                break;
            }
        }
        if (!running) break;

        // ── Advance mouse physics ─────────────────────────────────────────
        sim_mouse_update(dt_s);
        hal_tick(SIM_TIMESTEP_MS);

        // ── Navigation tick (only when at cell center) ────────────────────
        if (sim_mouse_arrived()) {
            SimNavResult result = sim_nav_step();

            status.total_steps++;

            switch (result) {
                case NAV_MOVED:
                    break;

                case NAV_PHASE_GOAL_REACHED:
                    printf("[sim] Run %d: reached goal. Steps: %d\n",
                           status.run_number, status.total_steps);
                    break;

                case NAV_PHASE_START_REACHED:
                    printf("[sim] Run %d: back at start.\n", status.run_number);
                    status.run_number++;
                    status.phase = SIM_PHASE_EXPLORING;
                    break;

                case NAV_DONE:
                    printf("[sim] Maze solved. Best path proven.\n");
                    status.phase       = SIM_PHASE_DONE;
                    status.maze_solved = true;
                    // Keep rendering — let user see result until window closed
                    break;

                case NAV_ERROR:
                    printf("[sim] Navigation error. Halting.\n");
                    running = false;
                    break;
            }
        }

        // ── Render current frame ──────────────────────────────────────────
        sim_render_frame(&status);

        // ── Frame pacing — cap to ~60fps wall clock ───────────────────────
        uint32_t elapsed = SDL_GetTicks() - frame_start;
        if (elapsed < SIM_TIMESTEP_MS)
            SDL_Delay(SIM_TIMESTEP_MS - elapsed);
    }

    sim_render_shutdown();
    hal_shutdown();
    printf("[sim] Simulation ended. Total steps: %d\n", status.total_steps);
}

const SimStatus *sim_loop_get_status(void) {
    return &status;
}