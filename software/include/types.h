#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

#define MAZE_SIZE 16

typedef enum {
    DIR_NORTH,
    DIR_EAST,
    DIR_SOUTH,
    DIR_WEST
} Direction;

typedef struct {
    float distance_mm;
    float heading_deg;
    uint8_t wall_flags;
} SensorFrame;

#endif
