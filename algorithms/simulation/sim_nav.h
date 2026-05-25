#ifndef SIM_NAV_H
#define SIM_NAV_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_nav.h  —  Navigation adapter
//
// Bridges the sim_loop (physical world) and the core algorithm (planner,
// floodfill, belief_maze). This is the sim-side equivalent of state.c —
// it owns the sense → plan → act cycle, but driven by the sim loop's
// arrival events rather than a blocking while-loop.
//
// Core logic files (floodfill.c, planner.c, belief_maze.c) are called
// here and nowhere else in the simulation folder.
// ─────────────────────────────────────────────────────────────────────────────

typedef enum {
    NAV_MOVED,               // Normal step, mouse issued a move command
    NAV_PHASE_GOAL_REACHED,  // Just arrived at goal for the first time this run
    NAV_PHASE_START_REACHED, // Just arrived back at start
    NAV_DONE,                // Optimistic == pessimistic, maze fully known
    NAV_ERROR                // Something went wrong
} SimNavResult;

// Initialize navigation state. Calls belief_maze_init internally.
void sim_nav_init(void);

// Called once per sim tick when mouse is at a cell center.
// Senses walls, replans, issues next move via sim_mouse_move_to_cell.
// Returns status so sim_loop can update its phase tracking.
SimNavResult sim_nav_step(void);

// Read-only: current proven best path (populated after maze solved)
void sim_nav_get_best_path(const uint8_t **rows, const uint8_t **cols, int *len);

#endif // SIM_NAV_H