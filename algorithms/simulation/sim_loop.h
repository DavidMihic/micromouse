#ifndef SIM_LOOP_H
#define SIM_LOOP_H

#include <stdbool.h>
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_loop.h  —  Simulation heartbeat
//
// Owns the fixed-timestep loop. Each tick:
//   1. Advances sim_mouse position
//   2. Checks if mouse arrived at cell center
//   3. If arrived: triggers sense → replan → issue next move command
//   4. Advances sim clock
//   5. Signals renderer to draw current frame
//
// The loop knows nothing about floodfill or maze logic. It only knows:
//   "has the mouse arrived?" and "what does the navigation layer want next?"
//
// The navigation layer (state.c equivalent for sim) is called exclusively
// through the step callback — the loop never reaches into planner directly.
// ─────────────────────────────────────────────────────────────────────────────

// Fixed simulation timestep (milliseconds)
#define SIM_TIMESTEP_MS 16

typedef enum {
    SIM_PHASE_EXPLORING,   // Mouse is mapping the maze
    SIM_PHASE_RETURNING,   // Mouse returning to start between runs
    SIM_PHASE_SPEEDRUN,    // Reserved — not implemented in this phase
    SIM_PHASE_DONE         // Maze fully mapped, best path proven
} SimPhase;

typedef struct {
    SimPhase phase;
    int      run_number;
    int      total_steps;
    bool     maze_solved;
} SimStatus;

// Initialize loop and all subsystems. Call once before sim_loop_run.
void sim_loop_init(void);

// Run the simulation. Blocks until window is closed or maze is solved.
void sim_loop_run(void);

// Read current simulation status
const SimStatus *sim_loop_get_status(void);

#endif // SIM_LOOP_H