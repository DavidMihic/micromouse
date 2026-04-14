#include "planner.h"
#include "floodfill.h"
#include "maze.h"
#include "belief_maze.h"
#include <stdint.h>

#define GOAL_COUNT 4
static const uint8_t goal_rows[GOAL_COUNT] = {7, 7, 8, 8};
static const uint8_t goal_cols[GOAL_COUNT] = {7, 8, 7, 8};

static Direction pick_next(const Mouse *mouse, bool (*has_wall)(uint8_t, uint8_t, Direction)) {
    uint8_t r = mouse->row;
    uint8_t c = mouse->col;

    uint8_t best_val = 255;
    Direction best_dir = DIR_NORTH;

    if (!has_wall(r, c, DIR_NORTH) && r + 1 < MAZE_SIZE && flood[r+1][c] < best_val) {
        best_val = flood[r+1][c];
        best_dir = DIR_NORTH;
    }
    if (!has_wall(r, c, DIR_SOUTH) && r > 0 && flood[r-1][c] < best_val) {
        best_val = flood[r-1][c];
        best_dir = DIR_SOUTH;
    }
    if (!has_wall(r, c, DIR_EAST) && c + 1 < MAZE_SIZE && flood[r][c+1] < best_val) {
        best_val = flood[r][c+1];
        best_dir = DIR_EAST;
    }
    if (!has_wall(r, c, DIR_WEST) && c > 0 && flood[r][c-1] < best_val) {
        best_val = flood[r][c-1];
        best_dir = DIR_WEST;
    }

    return best_dir;
}

Direction planner_next_move(const Mouse *mouse) {
    return pick_next(mouse, maze_has_wall);
}

Direction planner_next_move_belief(const Mouse *mouse) {
    return pick_next(mouse, belief_maze_has_wall);
}

Direction planner_next_move_belief_pessimistic(const Mouse *mouse) {
    return pick_next(mouse, belief_maze_has_wall_pessimistic);
}

void planner_move(Mouse *mouse, Direction dir) {
    switch (dir) {
        case DIR_NORTH: mouse->row += 1; break;
        case DIR_SOUTH: mouse->row -= 1; break;
        case DIR_EAST:  mouse->col += 1; break;
        case DIR_WEST:  mouse->col -= 1; break;
    }
    mouse->heading = dir;
}

int planner_at_goal(const Mouse *mouse) {
    for (int i = 0; i < GOAL_COUNT; i++)
        if (mouse->row == goal_rows[i] && mouse->col == goal_cols[i])
            return 1;
    return 0;
}

int planner_at_start(const Mouse *mouse) {
    return mouse->row == 0 && mouse->col == 0;
}