#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "types.h"
#include <stdbool.h>

extern uint8_t flood[MAZE_SIZE][MAZE_SIZE];

// Real maze — omniscient reference
void floodfill_compute(void);

// Belief optimistic — exploration
void floodfill_compute_on_belief(void);

// Belief pessimistic — speed run / best path check
void floodfill_compute_on_belief_pessimistic(void);

// Flood toward start (0,0) instead of goal — used for return trip
void floodfill_compute_return_on_belief(void);

void floodfill_print(void);

#endif