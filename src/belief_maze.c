#include "belief_maze.h"
#include <string.h>

static uint16_t b_north[MAZE_SIZE];
static uint16_t b_south[MAZE_SIZE];
static uint16_t b_east[MAZE_SIZE];
static uint16_t b_west[MAZE_SIZE];

static uint16_t k_north[MAZE_SIZE];
static uint16_t k_south[MAZE_SIZE];
static uint16_t k_east[MAZE_SIZE];
static uint16_t k_west[MAZE_SIZE];

void belief_maze_init(void) {
    memset(b_north, 0, sizeof(b_north));
    memset(b_south, 0, sizeof(b_south));
    memset(b_east,  0, sizeof(b_east));
    memset(b_west,  0, sizeof(b_west));
    memset(k_north, 0, sizeof(k_north));
    memset(k_south, 0, sizeof(k_south));
    memset(k_east,  0, sizeof(k_east));
    memset(k_west,  0, sizeof(k_west));

    // Outer boundary always known
    for (uint8_t i = 0; i < MAZE_SIZE; i++) {
        b_south[0]           |= (1 << i);
        k_south[0]           |= (1 << i);
        b_north[MAZE_SIZE-1] |= (1 << i);
        k_north[MAZE_SIZE-1] |= (1 << i);
        b_west[i]            |= (1 << 0);
        k_west[i]            |= (1 << 0);
        b_east[i]            |= (1 << (MAZE_SIZE-1));
        k_east[i]            |= (1 << (MAZE_SIZE-1));
    }
}

// Optimistic — unknown = open, draws mouse toward unexplored areas
bool belief_maze_has_wall(uint8_t row, uint8_t col, Direction dir) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return true;
    switch (dir) {
        case DIR_NORTH: return (b_north[row] >> col) & 1;
        case DIR_SOUTH: return (b_south[row] >> col) & 1;
        case DIR_EAST:  return (b_east[row]  >> col) & 1;
        case DIR_WEST:  return (b_west[row]  >> col) & 1;
        default:        return true;
    }
}

// Pessimistic — unknown = wall, only routes through proven corridors
bool belief_maze_has_wall_pessimistic(uint8_t row, uint8_t col, Direction dir) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return true;
    switch (dir) {
        case DIR_NORTH:
            if (!((k_north[row] >> col) & 1)) return true;
            return (b_north[row] >> col) & 1;
        case DIR_SOUTH:
            if (!((k_south[row] >> col) & 1)) return true;
            return (b_south[row] >> col) & 1;
        case DIR_EAST:
            if (!((k_east[row] >> col) & 1)) return true;
            return (b_east[row] >> col) & 1;
        case DIR_WEST:
            if (!((k_west[row] >> col) & 1)) return true;
            return (b_west[row] >> col) & 1;
        default: return true;
    }
}

void belief_maze_set_wall(uint8_t row, uint8_t col, Direction dir, bool present) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return;

    switch (dir) {
        case DIR_NORTH:
            if (present) b_north[row] |=  (1 << col);
            else         b_north[row] &= ~(1 << col);
            k_north[row] |= (1 << col);
            if (row + 1 < MAZE_SIZE) {
                if (present) b_south[row+1] |=  (1 << col);
                else         b_south[row+1] &= ~(1 << col);
                k_south[row+1] |= (1 << col);
            }
            break;
        case DIR_SOUTH:
            if (present) b_south[row] |=  (1 << col);
            else         b_south[row] &= ~(1 << col);
            k_south[row] |= (1 << col);
            if (row > 0) {
                if (present) b_north[row-1] |=  (1 << col);
                else         b_north[row-1] &= ~(1 << col);
                k_north[row-1] |= (1 << col);
            }
            break;
        case DIR_EAST:
            if (present) b_east[row] |=  (1 << col);
            else         b_east[row] &= ~(1 << col);
            k_east[row] |= (1 << col);
            if (col + 1 < MAZE_SIZE) {
                if (present) b_west[row] |=  (1 << (col+1));
                else         b_west[row] &= ~(1 << (col+1));
                k_west[row] |= (1 << (col+1));
            }
            break;
        case DIR_WEST:
            if (present) b_west[row] |=  (1 << col);
            else         b_west[row] &= ~(1 << col);
            k_west[row] |= (1 << col);
            if (col > 0) {
                if (present) b_east[row] |=  (1 << (col-1));
                else         b_east[row] &= ~(1 << (col-1));
                k_east[row] |= (1 << (col-1));
            }
            break;
    }
}

bool belief_maze_is_known(uint8_t row, uint8_t col, Direction dir) {
    if (row >= MAZE_SIZE || col >= MAZE_SIZE) return false;
    switch (dir) {
        case DIR_NORTH: return (k_north[row] >> col) & 1;
        case DIR_SOUTH: return (k_south[row] >> col) & 1;
        case DIR_EAST:  return (k_east[row]  >> col) & 1;
        case DIR_WEST:  return (k_west[row]  >> col) & 1;
        default:        return false;
    }
}