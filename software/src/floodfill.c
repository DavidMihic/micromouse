#include "floodfill.h"
#include "maze.h"
#include "belief_maze.h"
#include <stdio.h>

#define UNVISITED 255

#define GOAL_COUNT 4
static const uint8_t goal_rows[GOAL_COUNT] = {7, 7, 8, 8};
static const uint8_t goal_cols[GOAL_COUNT] = {7, 8, 7, 8};

uint8_t flood[MAZE_SIZE][MAZE_SIZE];

static uint8_t queue_row[MAZE_SIZE * MAZE_SIZE];
static uint8_t queue_col[MAZE_SIZE * MAZE_SIZE];

// Core BFS — seeds from given cells, uses given wall function
static void floodfill_run_seeded(
    bool (*has_wall)(uint8_t, uint8_t, Direction),
    const uint8_t *seed_rows,
    const uint8_t *seed_cols,
    int seed_count)
{
    for (uint8_t r = 0; r < MAZE_SIZE; r++)
        for (uint8_t c = 0; c < MAZE_SIZE; c++)
            flood[r][c] = UNVISITED;

    int head = 0, tail = 0;

    for (int i = 0; i < seed_count; i++) {
        uint8_t r = seed_rows[i];
        uint8_t c = seed_cols[i];
        flood[r][c] = 0;
        queue_row[tail] = r;
        queue_col[tail] = c;
        tail++;
    }

    while (head < tail) {
        uint8_t r = queue_row[head];
        uint8_t c = queue_col[head];
        head++;

        uint8_t next = flood[r][c] + 1;

        if (r + 1 < MAZE_SIZE && !has_wall(r, c, DIR_NORTH) && flood[r+1][c] == UNVISITED) {
            flood[r+1][c] = next;
            queue_row[tail] = r + 1;
            queue_col[tail] = c;
            tail++;
        }
        if (r > 0 && !has_wall(r, c, DIR_SOUTH) && flood[r-1][c] == UNVISITED) {
            flood[r-1][c] = next;
            queue_row[tail] = r - 1;
            queue_col[tail] = c;
            tail++;
        }
        if (c + 1 < MAZE_SIZE && !has_wall(r, c, DIR_EAST) && flood[r][c+1] == UNVISITED) {
            flood[r][c+1] = next;
            queue_row[tail] = r;
            queue_col[tail] = c + 1;
            tail++;
        }
        if (c > 0 && !has_wall(r, c, DIR_WEST) && flood[r][c-1] == UNVISITED) {
            flood[r][c-1] = next;
            queue_row[tail] = r;
            queue_col[tail] = c - 1;
            tail++;
        }
    }
}

void floodfill_compute(void) {
    floodfill_run_seeded(maze_has_wall, goal_rows, goal_cols, GOAL_COUNT);
}

void floodfill_compute_on_belief(void) {
    floodfill_run_seeded(belief_maze_has_wall, goal_rows, goal_cols, GOAL_COUNT);
}

void floodfill_compute_on_belief_pessimistic(void) {
    floodfill_run_seeded(belief_maze_has_wall_pessimistic, goal_rows, goal_cols, GOAL_COUNT);
}

void floodfill_compute_return_on_belief(void) {
    // Seed is (0,0) — navigate back to start
    static const uint8_t start_row[1] = {0};
    static const uint8_t start_col[1] = {0};
    floodfill_run_seeded(belief_maze_has_wall, start_row, start_col, 1);
}

void floodfill_print(void) {
    printf("\nFloodfill distance map (row 15 = top):\n\n");
    for (int r = MAZE_SIZE - 1; r >= 0; r--) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            if (flood[r][c] == UNVISITED)
                printf("  ? ");
            else
                printf("%3d ", flood[r][c]);
        }
        printf("\n");
    }
    printf("\n");
}