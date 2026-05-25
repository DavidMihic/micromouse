#ifndef SIM_RENDER_H
#define SIM_RENDER_H

#include "sim_loop.h"
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
// sim_render.h  —  SDL2 visualizer for the simulation
//
// Draws the maze, the mouse as a positioned and oriented triangle in
// continuous space, the belief map (known walls), visited cell trail,
// and a status overlay showing phase and step count.
//
// Completely separate from maze_visual.c — the original renderer is
// untouched. This one knows about continuous mouse position and belief state.
// ─────────────────────────────────────────────────────────────────────────────

// Initialize SDL window and renderer. Call once.
void sim_render_init(void);

// Draw one frame reflecting current sim state.
// Called every tick by sim_loop.
void sim_render_frame(const SimStatus *status);

// Tear down SDL. Call once on exit.
void sim_render_shutdown(void);

#endif // SIM_RENDER_H