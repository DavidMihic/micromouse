#ifndef MAZE_H
#define MAZE_H

#include "types.h"
#include "state.h"
#include <stdbool.h>

bool maze_has_wall(uint8_t row, uint8_t col, Direction dir);
bool maze_is_visited(uint8_t row, uint8_t col);
void maze_set_visited(uint8_t row, uint8_t col);
void maze_reset_visited(void);

void maze_render_window(void);

void maze_render_animated_full(
    ExplorationFrame *frames, int frame_count,
    uint8_t *best_rows, uint8_t *best_cols, int best_len);

#endif