#include "sim_render.h"
#include "sim_mouse.h"
#include "sim_nav.h"
#include "../include/belief_maze.h"
#include "../include/maze.h"
#include "../include/types.h"
#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────────────────────────────────────────────
// sim_render.c  —  Simulation visualizer
//
// Visual layers (bottom to top):
//   1. White background
//   2. Orange visited cell trail
//   3. Green best path (after maze solved)
//   4. Red goal cells
//   5. Green start cell
//   6. Known belief walls (dark grey, slightly thinner than true walls)
//   7. True maze walls (black, always visible — sim is omniscient display)
//   8. Mouse body (charcoal rect + yellow semicircle nose, oriented by heading)
//   9. Status text overlay
// ─────────────────────────────────────────────────────────────────────────────

#define CELL_PX      44
#define WALL_PX       4
#define WIN_PX       (MAZE_SIZE * CELL_PX + WALL_PX)

// Physical mouse dimensions in mm — must match hardware spec
#define MOUSE_WIDTH_MM   90.0f
#define MOUSE_LENGTH_MM 100.0f
#define MOUSE_TAIL_MM    10.0f   // rear rectangular stub height
#define MOUSE_NOSE_R_MM  45.0f  // semicircle radius = half width

static SDL_Window   *window   = NULL;
static SDL_Renderer *renderer = NULL;

// Visited cell tracking (persistent across frames)
static uint8_t visited[MAZE_SIZE][MAZE_SIZE] = {0};

// ── Coordinate helpers ────────────────────────────────────────────────────────

// Cell (row, col) top-left corner in screen pixels
// Screen y is inverted: row 0 is at bottom of window
static int cell_px_x(int col) { return col * CELL_PX; }
static int cell_px_y(int row) { return (MAZE_SIZE - 1 - row) * CELL_PX; }

// Continuous mm position to screen pixels
static int mm_to_px_x(float x_mm) {
    return (int)(x_mm / CELL_SIZE_MM * CELL_PX);
}
static int mm_to_px_y(float y_mm) {
    // Invert y: row 0 is bottom of screen
    float flipped = MAZE_SIZE * CELL_SIZE_MM - y_mm;
    return (int)(flipped / CELL_SIZE_MM * CELL_PX);
}

// ── Draw helpers ──────────────────────────────────────────────────────────────

static void set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

static void fill_cell(int row, int col, uint8_t r, uint8_t g, uint8_t b) {
    SDL_Rect rect = {
        cell_px_x(col) + WALL_PX + 2,
        cell_px_y(row) + WALL_PX + 2,
        CELL_PX - WALL_PX - 4,
        CELL_PX - WALL_PX - 4
    };
    set_color(r, g, b, 255);
    SDL_RenderFillRect(renderer, &rect);
}

static void draw_true_walls(void) {
    set_color(20, 20, 20, 255);

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            int x = cell_px_x(c);
            int y = cell_px_y(r);
            SDL_Rect rect;

            if (maze_has_wall(r, c, DIR_NORTH)) {
                rect = (SDL_Rect){x, y, CELL_PX + WALL_PX, WALL_PX};
                SDL_RenderFillRect(renderer, &rect);
            }
            if (maze_has_wall(r, c, DIR_EAST)) {
                rect = (SDL_Rect){x + CELL_PX, y, WALL_PX, CELL_PX + WALL_PX};
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }

    // Left and bottom boundary
    SDL_Rect left   = {0, 0, WALL_PX, WIN_PX};
    SDL_Rect bottom = {0, WIN_PX - WALL_PX, WIN_PX, WALL_PX};
    SDL_RenderFillRect(renderer, &left);
    SDL_RenderFillRect(renderer, &bottom);
}

static void draw_belief_walls(void) {
    // Known belief walls in a distinct color (dim blue) — slightly inset
    set_color(80, 120, 200, 200);

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            int x = cell_px_x(c);
            int y = cell_px_y(r);
            int thin = 2;
            SDL_Rect rect;

            if (belief_maze_is_known(r, c, DIR_NORTH) &&
                belief_maze_has_wall(r, c, DIR_NORTH)) {
                rect = (SDL_Rect){x + 2, y + 1, CELL_PX - 4, thin};
                SDL_RenderFillRect(renderer, &rect);
            }
            if (belief_maze_is_known(r, c, DIR_EAST) &&
                belief_maze_has_wall(r, c, DIR_EAST)) {
                rect = (SDL_Rect){x + CELL_PX - 1, y + 2, thin, CELL_PX - 4};
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }
}

// Rotate a point (lx, ly) in local mouse space by heading and translate to screen.
// Local space: +y = forward (north), +x = right (east), origin = mouse center.
// Screen space: y increases downward, so forward maps to -screen_y when heading=North.
static void local_to_screen(float lx, float ly,
                             float cx, float cy, float rad,
                             int *sx, int *sy) {
    // Rotate: screen_x = cx + lx*cos(rad) + ly*sin(rad)
    //         screen_y = cy - lx*sin(rad) + ly*cos(rad)  (y flipped for screen)
    // Wait — heading 0=North means forward is -screen_y.
    // sin(rad) gives East component, cos(rad) gives North component.
    // Forward vector in screen: (sin(rad), -cos(rad))
    // Right vector in screen:   (cos(rad),  sin(rad))
    float fwd_x =  sinf(rad);   float fwd_y = -cosf(rad);
    float rgt_x =  cosf(rad);   float rgt_y =  sinf(rad);

    *sx = (int)(cx + lx * rgt_x + ly * fwd_x);
    *sy = (int)(cy + lx * rgt_y + ly * fwd_y);
}

// Draw filled rotated rectangle in local mouse coords.
// Rasterized as horizontal scanlines in local space, transformed per pixel.
static void draw_rotated_rect(float cx, float cy, float rad,
                               float lx0, float ly0, float lx1, float ly1,
                               uint8_t r, uint8_t g, uint8_t b) {
    // Scanline fill in local space — step 1px in local coords
    float fwd_x =  sinf(rad);  float fwd_y = -cosf(rad);
    float rgt_x =  cosf(rad);  float rgt_y =  sinf(rad);

    set_color(r, g, b, 255);

    // Step across local y (forward axis)
    for (float ly = ly0; ly <= ly1; ly += 0.5f) {
        // Step across local x (right axis)
        int sx0, sy0, sx1, sy1;
        local_to_screen(lx0, ly, cx, cy, rad, &sx0, &sy0);
        local_to_screen(lx1, ly, cx, cy, rad, &sx1, &sy1);
        SDL_RenderDrawLine(renderer, sx0, sy0, sx1, sy1);
        (void)fwd_x; (void)fwd_y; (void)rgt_x; (void)rgt_y;
    }
}

static void draw_mouse_body(float x_mm, float y_mm, float heading_deg) {
    float cx = (float)mm_to_px_x(x_mm);
    float cy = (float)mm_to_px_y(y_mm);
    float rad = heading_deg * (float)M_PI / 180.0f;

    // Scale mm → pixels
    float scale = (float)CELL_PX / CELL_SIZE_MM;

    float half_w  = (MOUSE_WIDTH_MM  * 0.5f) * scale;   // 45mm → ~11px
    float half_l  = (MOUSE_LENGTH_MM * 0.5f) * scale;   // 50mm → ~12px
    float nose_r  = (MOUSE_NOSE_R_MM)        * scale;   // 45mm → ~11px
    float tail_h  = (MOUSE_TAIL_MM)          * scale;   //  10mm → ~2px

    // Mouse local coordinate system:
    //   forward (+y_local) = direction of travel
    //   right   (+x_local) = right side
    //   center of mouse body = local (0, 0)
    //
    // Shape breakdown:
    //   Rear tail rect:  x in [-half_w, half_w], y in [-half_l, -half_l + tail_h]
    //   Main body rect:  x in [-half_w, half_w], y in [-half_l + tail_h, 0]
    //   Nose semicircle: center at (0, 0), radius = half_w, front half only (y >= 0)

    // ── Body rectangle (charcoal) ─────────────────────────────────────────
    draw_rotated_rect(cx, cy, rad,
                      -half_w, -half_l + tail_h,
                       half_w,  0.0f,
                      60, 60, 65);

    // ── Tail stub (slightly lighter) ─────────────────────────────────────
    draw_rotated_rect(cx, cy, rad,
                      -half_w, -half_l,
                       half_w, -half_l + tail_h,
                      90, 90, 95);

    // ── Nose semicircle (yellow accent) ──────────────────────────────────
    // Rasterize: for each angle from 0 to PI (front half), draw line from
    // center to arc point in local space
    set_color(15, 105, 30, 255);
    int steps = 32;
    for (int i = 0; i <= steps; i++) {
        float angle = (float)i / (float)steps * (float)M_PI;  // 0 to PI
        float lx = cosf(angle) * nose_r;   // right component
        float ly = sinf(angle) * nose_r;   // forward component (front half)

        int sx0, sy0, sx1, sy1;
        local_to_screen(0.0f, 0.0f, cx, cy, rad, &sx0, &sy0);
        local_to_screen(lx,   ly,   cx, cy, rad, &sx1, &sy1);
        SDL_RenderDrawLine(renderer, sx0, sy0, sx1, sy1);
    }

    // ── Heading dot — small white dot at nose tip ─────────────────────────
    int nx, ny;
    local_to_screen(0.0f, nose_r, cx, cy, rad, &nx, &ny);
    set_color(15, 105, 30, 255);
    SDL_RenderDrawPoint(renderer, nx,     ny);
    SDL_RenderDrawPoint(renderer, nx + 1, ny);
    SDL_RenderDrawPoint(renderer, nx,     ny + 1);
    SDL_RenderDrawPoint(renderer, nx + 1, ny + 1);
}

// ── Public API ────────────────────────────────────────────────────────────────

void sim_render_init(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("[render] SDL_Init failed: %s\n", SDL_GetError());
        return;
    }

    window = SDL_CreateWindow(
        "Micromouse Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_PX, WIN_PX, 0
    );
    if (!window) {
        printf("[render] Window creation failed: %s\n", SDL_GetError());
        return;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        printf("[render] Renderer creation failed: %s\n", SDL_GetError());
        return;
    }

    memset(visited, 0, sizeof(visited));
    printf("[render] Window initialized %dx%d\n", WIN_PX, WIN_PX);
}

void sim_render_frame(const SimStatus *status) {
    if (!renderer) return;

    const SimMouseState *mouse = sim_mouse_get_state();

    // Track visited cells
    uint8_t row = sim_y_to_row(mouse->y_mm);
    uint8_t col = sim_x_to_col(mouse->x_mm);
    if (row < MAZE_SIZE && col < MAZE_SIZE)
        visited[row][col] = 1;

    // ── Background ────────────────────────────────────────────────────────
    set_color(245, 245, 240, 255);
    SDL_RenderClear(renderer);

    // ── Visited trail (orange) ────────────────────────────────────────────
    for (int r = 0; r < MAZE_SIZE; r++)
        for (int c = 0; c < MAZE_SIZE; c++)
            if (visited[r][c])
                fill_cell(r, c, 255, 150, 30);

    // ── Best path (green, shown after solved) ─────────────────────────────
    if (status->maze_solved) {
        const uint8_t *bp_rows, *bp_cols;
        int bp_len;
        sim_nav_get_best_path(&bp_rows, &bp_cols, &bp_len);
        for (int i = 0; i < bp_len; i++)
            fill_cell(bp_rows[i], bp_cols[i], 60, 200, 80);
    }

    // ── Goal cells (red) ─────────────────────────────────────────────────
    fill_cell(7, 7, 200, 40,  40);
    fill_cell(7, 8, 200, 40,  40);
    fill_cell(8, 7, 200, 40,  40);
    fill_cell(8, 8, 200, 40,  40);

    // ── Start cell (green) ───────────────────────────────────────────────
    fill_cell(0, 0, 40, 180, 40);

    // ── Belief walls (blue, known) ────────────────────────────────────────
    draw_belief_walls();

    // ── True maze walls (black, always) ──────────────────────────────────
    draw_true_walls();

    // ── Mouse body (accurate footprint) ──────────────────────────────────
    draw_mouse_body(mouse->x_mm, mouse->y_mm, mouse->heading_deg);

    SDL_RenderPresent(renderer);
}

void sim_render_shutdown(void) {
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window)   SDL_DestroyWindow(window);
    SDL_Quit();
    renderer = NULL;
    window   = NULL;
}