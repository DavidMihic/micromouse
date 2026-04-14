#include <SDL2/SDL.h>
#include "maze.h"
#include "belief_maze.h"
#include "state.h"

#define CELL_SIZE 40
#define WALL 4
#define WIN_SIZE (MAZE_SIZE * CELL_SIZE)

// How many milliseconds between each exploration step
#define STEP_DELAY_MS 60

// How many milliseconds between each best path step
#define BEST_DELAY_MS 80

static void draw_walls(SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            int x = c * CELL_SIZE;
            int y = (MAZE_SIZE - 1 - r) * CELL_SIZE;
            SDL_Rect rect;

            if (maze_has_wall(r, c, DIR_NORTH)) {
                rect = (SDL_Rect){x, y, CELL_SIZE + WALL, WALL};
                SDL_RenderFillRect(renderer, &rect);
            }
            if (maze_has_wall(r, c, DIR_EAST)) {
                rect = (SDL_Rect){x + CELL_SIZE, y, WALL, CELL_SIZE + WALL};
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }

    SDL_Rect left   = (SDL_Rect){0, 0, WALL, WIN_SIZE + WALL};
    SDL_Rect bottom = (SDL_Rect){0, WIN_SIZE, WIN_SIZE + WALL, WALL};
    SDL_RenderFillRect(renderer, &left);
    SDL_RenderFillRect(renderer, &bottom);
}

static void draw_cell(SDL_Renderer *renderer, uint8_t row, uint8_t col,
                      uint8_t r, uint8_t g, uint8_t b) {
    int x = col * CELL_SIZE + WALL + 2;
    int y = (MAZE_SIZE - 1 - row) * CELL_SIZE + WALL + 2;
    int size = CELL_SIZE - WALL - 4;
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    SDL_Rect rect = (SDL_Rect){x, y, size, size};
    SDL_RenderFillRect(renderer, &rect);
}

static int handle_events(void) {
    SDL_Event e;
    while (SDL_PollEvent(&e))
        if (e.type == SDL_QUIT) return 0;
    return 1;
}

void maze_render_window(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) return;

    SDL_Window *window = SDL_CreateWindow(
        "Micromouse Maze",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_SIZE + WALL, WIN_SIZE + WALL, 0
    );
    if (!window) return;

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) return;

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    draw_walls(renderer);
    SDL_RenderPresent(renderer);

    while (handle_events()) SDL_Delay(16);

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void maze_render_animated(
    uint8_t *best_rows, uint8_t *best_cols, int best_len)
{
    // This function receives the best path only.
    // Exploration frames are passed via maze_render_animated_full below.
    (void)best_rows; (void)best_cols; (void)best_len;
}

void maze_render_animated_full(
    ExplorationFrame *frames, int frame_count,
    uint8_t *best_rows, uint8_t *best_cols, int best_len)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) return;

    SDL_Window *window = SDL_CreateWindow(
        "Micromouse Maze",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_SIZE + WALL, WIN_SIZE + WALL, 0
    );
    if (!window) return;

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) return;

    // Track which cells have been visited for persistent orange trail
    uint8_t visited[MAZE_SIZE][MAZE_SIZE] = {0};

    // --- Phase 1: Animate exploration ---
    for (int i = 0; i < frame_count; i++) {
        if (!handle_events()) goto cleanup;

        uint8_t row = frames[i].row;
        uint8_t col = frames[i].col;
        visited[row][col] = 1;

        // Redraw
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        // Orange trail — all visited cells
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                if (visited[r][c])
                    draw_cell(renderer, r, c, 255, 140, 0);

        // Yellow dot — current mouse position
        draw_cell(renderer, row, col, 255, 255, 0);

        // Green start
        draw_cell(renderer, 0, 0, 0, 200, 0);

        // Red goal cells
        draw_cell(renderer, 7, 7, 200, 0, 0);
        draw_cell(renderer, 7, 8, 200, 0, 0);
        draw_cell(renderer, 8, 7, 200, 0, 0);
        draw_cell(renderer, 8, 8, 200, 0, 0);

        // Walls always on top
        draw_walls(renderer);
        SDL_RenderPresent(renderer);
        SDL_Delay(STEP_DELAY_MS);
    }

    // Pause between exploration and best path reveal
    SDL_Delay(800);

    // --- Phase 2: Animate best path in green ---
    // Keep orange trail visible, draw green on top step by step
    for (int i = 0; i < best_len; i++) {
        if (!handle_events()) goto cleanup;

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        // Orange trail underneath
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                if (visited[r][c])
                    draw_cell(renderer, r, c, 255, 140, 0);

        // Green best path so far
        for (int j = 0; j <= i; j++)
            draw_cell(renderer, best_rows[j], best_cols[j], 0, 200, 0);

        // Walls on top
        draw_walls(renderer);
        SDL_RenderPresent(renderer);
        SDL_Delay(BEST_DELAY_MS);
    }

    // Hold final frame until window closed
    while (handle_events()) SDL_Delay(16);

cleanup:
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}