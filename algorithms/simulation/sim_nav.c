#include "sim_nav.h"
#include "sim_hal.h"
#include "sim_mouse.h"
#include "../include/belief_maze.h"
#include "../include/floodfill.h"
#include "../include/planner.h"
#include "../include/maze.h"
#include <string.h>
#include <stdio.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_nav.c  —  Sense → Plan → Act, one cell at a time
//
// Mirrors the logic of state.c but structured as a state machine driven
// by arrival events from sim_loop, rather than a blocking loop.
//
// Phases:
//   EXPLORE  — run to goal using optimistic belief floodfill
//   RETURN   — run back to start using belief return floodfill
//   DONE     — optimistic == pessimistic, best path proven and stored
// ─────────────────────────────────────────────────────────────────────────────

typedef enum {
    NAVSTATE_EXPLORE,
    NAVSTATE_RETURN,
    NAVSTATE_DONE
} NavState;

static NavState    nav_state = NAVSTATE_EXPLORE;
static Mouse       nav_mouse = {0, 0, DIR_NORTH};
static bool        first_step = true;

#define MAX_BEST_PATH 512
static uint8_t best_rows[MAX_BEST_PATH];
static uint8_t best_cols[MAX_BEST_PATH];
static int     best_len = 0;

// ── Internal sense — mirrors state.c sense() ─────────────────────────────────
static void sense_current_cell(void) {
    uint8_t r = nav_mouse.row;
    uint8_t c = nav_mouse.col;
    Direction heading = nav_mouse.heading;

    Direction front, left, right;
    switch (heading) {
        case DIR_NORTH: front=DIR_NORTH; left=DIR_WEST;  right=DIR_EAST;  break;
        case DIR_SOUTH: front=DIR_SOUTH; left=DIR_EAST;  right=DIR_WEST;  break;
        case DIR_EAST:  front=DIR_EAST;  left=DIR_NORTH; right=DIR_SOUTH; break;
        case DIR_WEST:  front=DIR_WEST;  left=DIR_SOUTH; right=DIR_NORTH; break;
        default: return;
    }

    Direction dirs[3] = {front, left, right};
    for (int i = 0; i < 3; i++) {
        Direction d = dirs[i];
        if (!belief_maze_is_known(r, c, d)) {
            bool wall = maze_has_wall(r, c, d);
            belief_maze_set_wall(r, c, d, wall);
        }
    }
}

static int optimistic_distance(void) {
    floodfill_compute_on_belief();
    return flood[0][0];
}

static int pessimistic_distance(void) {
    floodfill_compute_on_belief_pessimistic();
    return flood[0][0];
}

static void build_best_path(void) {
    floodfill_compute_on_belief_pessimistic();
    Mouse replay = {0, 0, DIR_NORTH};
    best_len = 0;

    best_rows[best_len] = replay.row;
    best_cols[best_len] = replay.col;
    best_len++;

    while (!planner_at_goal(&replay) && best_len < MAX_BEST_PATH) {
        Direction next = planner_next_move_belief_pessimistic(&replay);
        planner_move(&replay, next);
        best_rows[best_len] = replay.row;
        best_cols[best_len] = replay.col;
        best_len++;
    }

    printf("[nav] Best path length: %d steps\n", best_len - 1);
}

// ── Public API ────────────────────────────────────────────────────────────────

void sim_nav_init(void) {
    belief_maze_init();
    nav_mouse.row     = 0;
    nav_mouse.col     = 0;
    nav_mouse.heading = DIR_NORTH;
    nav_state = NAVSTATE_EXPLORE;
    first_step = true;
    best_len   = 0;

    floodfill_compute_on_belief();
}

SimNavResult sim_nav_step(void) {
    // Sync nav_mouse position from continuous sim state
    const SimMouseState *phys = sim_mouse_get_state();
    nav_mouse.row = sim_y_to_row(phys->y_mm);
    nav_mouse.col = sim_x_to_col(phys->x_mm);

    // Sense walls at current position
    sense_current_cell();

    if (nav_state == NAVSTATE_EXPLORE) {
        // Check if we've arrived at the goal
        if (planner_at_goal(&nav_mouse)) {
            int opt  = optimistic_distance();
            int pess = pessimistic_distance();
            printf("[nav] At goal. opt=%d pess=%d\n", opt, pess);

            if (opt == pess) {
                build_best_path();
                nav_state = NAVSTATE_DONE;
                return NAV_DONE;
            }

            // Need more exploration — return to start
            floodfill_compute_return_on_belief();
            nav_state = NAVSTATE_RETURN;
            return NAV_PHASE_GOAL_REACHED;
        }

        // Replan and move
        floodfill_compute_on_belief();
        Direction next = planner_next_move_belief(&nav_mouse);
        planner_move(&nav_mouse, next);
        sim_mouse_move_to_cell(nav_mouse.row, nav_mouse.col);
        return NAV_MOVED;
    }

    if (nav_state == NAVSTATE_RETURN) {
        if (planner_at_start(&nav_mouse)) {
            // Back at start — go for goal again
            floodfill_compute_on_belief();
            nav_state = NAVSTATE_EXPLORE;
            return NAV_PHASE_START_REACHED;
        }

        floodfill_compute_return_on_belief();
        Direction next = planner_next_move_belief(&nav_mouse);
        planner_move(&nav_mouse, next);
        sim_mouse_move_to_cell(nav_mouse.row, nav_mouse.col);
        return NAV_MOVED;
    }

    if (nav_state == NAVSTATE_DONE) {
        return NAV_DONE;
    }

    return NAV_ERROR;
}

void sim_nav_get_best_path(const uint8_t **rows, const uint8_t **cols, int *len) {
    if (rows) *rows = best_rows;
    if (cols) *cols = best_cols;
    if (len)  *len  = best_len;
}