#ifndef BELIEF_MAZE_H
#define BELIEF_MAZE_H

#include "types.h"
#include <stdbool.h>

// Initialize belief maze — only outer boundary walls known
void belief_maze_init(void);

// Optimistic — unknown walls treated as open
// Used during exploration so mouse is drawn toward unknown territory
bool belief_maze_has_wall(uint8_t row, uint8_t col, Direction dir);

// Pessimistic — unknown walls treated as solid
// Used for best path replay so mouse only travels proven corridors
bool belief_maze_has_wall_pessimistic(uint8_t row, uint8_t col, Direction dir);

// Write a wall into the belief maze
void belief_maze_set_wall(uint8_t row, uint8_t col, Direction dir, bool present);

// Check if a wall has been observed yet
bool belief_maze_is_known(uint8_t row, uint8_t col, Direction dir);

#endif