#ifndef PLANNER_H
#define PLANNER_H

#include "types.h"
#include <stdbool.h>

typedef struct {
    uint8_t row;
    uint8_t col;
    Direction heading;
} Mouse;

// Next move on real maze
Direction planner_next_move(const Mouse *mouse);

// Next move on belief maze optimistic — exploration
Direction planner_next_move_belief(const Mouse *mouse);

// Next move on belief maze pessimistic — speed run
Direction planner_next_move_belief_pessimistic(const Mouse *mouse);

// Move mouse one step
void planner_move(Mouse *mouse, Direction dir);

// Check if at goal center
int planner_at_goal(const Mouse *mouse);

// Check if at start
int planner_at_start(const Mouse *mouse);

#endif