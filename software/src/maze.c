#include "maze.h"

// Competition maze 2025 - wall data parsed from labirint2025.txt
// Each uint16_t is a bitmask for one row, bit N = column N has a wall

static const uint16_t north_walls[MAZE_SIZE] = {
    0x6D98, // row 0
    0x0C64, // row 1
    0x3730, // row 2
    0x48CA, // row 3
    0x3531, // row 4
    0xC366, // row 5
    0x3DD8, // row 6
    0x5035, // row 7
    0x0518, // row 8
    0x50A6, // row 9
    0x0B98, // row 10
    0x60C2, // row 11
    0x162C, // row 12
    0x594C, // row 13
    0x3632, // row 14
    0xFFFF, // row 15 - top boundary, all walls
};

static const uint16_t south_walls[MAZE_SIZE] = {
    0xFFFF, // row 0 - bottom boundary, all walls
    0x6D98, // row 1
    0x0C64, // row 2
    0x3730, // row 3
    0x48CA, // row 4
    0x3531, // row 5
    0xC366, // row 6
    0x3DD8, // row 7
    0x5035, // row 8
    0x0518, // row 9
    0x50A6, // row 10
    0x0B98, // row 11
    0x60C2, // row 12
    0x162C, // row 13
    0x594C, // row 14
    0x3632, // row 15
};

static const uint16_t east_walls[MAZE_SIZE] = {
    0x8401, // row 0
    0xF0AB, // row 1
    0x8512, // row 2
    0xC845, // row 3
    0xDB55, // row 4
    0x8B02, // row 5
    0xD6A8, // row 6
    0xAD73, // row 7
    0xAB43, // row 8
    0xD268, // row 9
    0xD56A, // row 10
    0xA749, // row 11
    0xAA15, // row 12
    0xCAD0, // row 13
    0xC2D5, // row 14
    0x9001, // row 15
};

static const uint16_t west_walls[MAZE_SIZE] = {
    0x0803, // row 0
    0xE157, // row 1
    0x0A25, // row 2
    0x908B, // row 3
    0xB6AB, // row 4
    0x1605, // row 5
    0xAD51, // row 6
    0x5AE7, // row 7
    0x5687, // row 8
    0xA4D1, // row 9
    0xAAD5, // row 10
    0x4E93, // row 11
    0x542B, // row 12
    0x95A1, // row 13
    0x85AB, // row 14
    0x2003, // row 15
};

// Visited flags - one bit per cell, 16 bits per row
static uint16_t visited[MAZE_SIZE] = {0};

bool maze_has_wall(uint8_t row, uint8_t col, Direction dir) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return true;
    switch (dir) {
        case DIR_NORTH: return (north_walls[row] >> col) & 1;
        case DIR_SOUTH: return (south_walls[row] >> col) & 1;
        case DIR_EAST:  return (east_walls[row]  >> col) & 1;
        case DIR_WEST:  return (west_walls[row]  >> col) & 1;
        default:        return true;
    }
}

bool maze_is_visited(uint8_t row, uint8_t col) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return false;
    return (visited[row] >> col) & 1;
}

void maze_set_visited(uint8_t row, uint8_t col) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return;
    visited[row] |= (1 << col);
}

void maze_reset_visited(void) {
    for (uint8_t i = 0; i < MAZE_SIZE; i++) visited[i] = 0;
}