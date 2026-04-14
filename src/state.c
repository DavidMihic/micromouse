#include "state.h"
#include "belief_maze.h"
#include "maze.h"
#include "floodfill.h"
#include "planner.h"
#include <stdio.h>

static void sense(Mouse *mouse) {
    uint8_t r = mouse->row;
    uint8_t c = mouse->col;
    Direction heading = mouse->heading;

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

static int run_to_goal(Mouse *mouse,
                       ExplorationFrame *frames, int *frame_count) {
    int steps = 0;
    floodfill_compute_on_belief();

    while (!planner_at_goal(mouse) && steps < 512) {
        sense(mouse);
        floodfill_compute_on_belief();
        Direction next = planner_next_move_belief(mouse);
        planner_move(mouse, next);
        steps++;

        // Record frame
        frames[*frame_count].row = mouse->row;
        frames[*frame_count].col = mouse->col;
        (*frame_count)++;
    }
    sense(mouse);
    return steps;
}

static int run_to_start(Mouse *mouse,
                        ExplorationFrame *frames, int *frame_count) {
    int steps = 0;
    floodfill_compute_return_on_belief();

    while (!planner_at_start(mouse) && steps < 512) {
        sense(mouse);
        floodfill_compute_return_on_belief();
        Direction next = planner_next_move_belief(mouse);
        planner_move(mouse, next);
        steps++;

        frames[*frame_count].row = mouse->row;
        frames[*frame_count].col = mouse->col;
        (*frame_count)++;
    }
    sense(mouse);
    return steps;
}

static int optimistic_best(void) {
    floodfill_compute_on_belief();
    return flood[0][0];
}

static int pessimistic_best(void) {
    floodfill_compute_on_belief_pessimistic();
    return flood[0][0];
}

int state_run_full(
    Mouse *mouse,
    ExplorationFrame *frames, int *frame_count,
    uint8_t *best_rows, uint8_t *best_cols, int *best_len)
{
    belief_maze_init();
    *frame_count = 0;

    // Record starting cell
    frames[*frame_count].row = mouse->row;
    frames[*frame_count].col = mouse->col;
    (*frame_count)++;

    int run = 0;
    int total_steps = 0;

    while (1) {
        run++;
        printf("Run %d: going to goal...\n", run);
        total_steps += run_to_goal(mouse, frames, frame_count);
        printf("  reached goal. total steps: %d\n", total_steps);

        int opt  = optimistic_best();
        int pess = pessimistic_best();
        printf("  optimistic: %d  pessimistic: %d\n", opt, pess);

        if (opt == pess) {
            printf("  paths agree — done.\n");
            break;
        }

        printf("Run %d: returning to start...\n", run);
        total_steps += run_to_start(mouse, frames, frame_count);
        printf("  back at start.\n");
    }

    // Build proven best path
    floodfill_compute_on_belief_pessimistic();

    Mouse replay = {0, 0, DIR_NORTH};
    *best_len = 0;

    best_rows[*best_len] = replay.row;
    best_cols[*best_len] = replay.col;
    (*best_len)++;

    while (!planner_at_goal(&replay) && *best_len < 512) {
        Direction next = planner_next_move_belief_pessimistic(&replay);
        planner_move(&replay, next);
        best_rows[*best_len] = replay.row;
        best_cols[*best_len] = replay.col;
        (*best_len)++;
    }

    return total_steps;
}