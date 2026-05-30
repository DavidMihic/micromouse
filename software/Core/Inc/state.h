#ifndef STATE_H
#define STATE_H

#include "types.h"
#include "planner.h"
#include <stdint.h>

// Snapshot of mouse state at one point in time
// Used to replay exploration frame by frame in the visual
typedef struct {
    uint8_t row;
    uint8_t col;

    // Belief maze walls known at this moment
    // We store the full visited cell trail only —
    // belief maze is rebuilt at render time from state.c
} ExplorationFrame;

// Run full exploration and return trips until maze is mapped
// Fills frames array with every step the mouse took
// Returns proven shortest path length
int state_run_full(
    Mouse *mouse,
    ExplorationFrame *frames, int *frame_count,
    uint8_t *best_rows, uint8_t *best_cols, int *best_len);

#endif