#include <stdio.h>
#include "types.h"
#include "maze.h"
#include "belief_maze.h"
#include "floodfill.h"
#include "planner.h"
#include "state.h"

// Max frames — exploration can take many steps across multiple runs
#define MAX_FRAMES 2048

int main(void) {
    Mouse mouse = {0, 0, DIR_NORTH};

    static ExplorationFrame frames[MAX_FRAMES];
    int frame_count = 0;

    uint8_t best_rows[512];
    uint8_t best_cols[512];
    int best_len = 0;

    int total = state_run_full(
        &mouse,
        frames, &frame_count,
        best_rows, best_cols, &best_len);

    printf("\nTotal exploration steps: %d\n", total);
    printf("Proven shortest path:    %d steps.\n", best_len - 1);
    printf("Exploration frames:      %d\n", frame_count);

    maze_render_animated_full(
        frames, frame_count,
        best_rows, best_cols, best_len);

    return 0;
}