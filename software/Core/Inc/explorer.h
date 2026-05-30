/*
 * explorer.h
 *
 * Firmware glue between the navigator (motion/heading/pose) and the maze
 * solving algorithm (belief_maze + floodfill + planner).
 *
 * Coordinate bridge:
 *   planner row  = navigator cell_y (north axis)
 *   planner col  = navigator cell_x (east axis)
 *   Direction    == Heading (same enum values N,E,S,W)
 *
 *  Created on: May 30, 2026.
 *      Author: david
 */

#ifndef INC_EXPLORER_H_
#define INC_EXPLORER_H_

#include <stdbool.h>

#include "navigator.h"
#include "planner.h"   /* Mouse, Direction */

typedef enum
{
    EXPLORER_IDLE,
    EXPLORER_EXPLORE,   /* optimistic flood toward the goal      */
    EXPLORER_RETURN,    /* flood back toward the start           */
    EXPLORER_SPEED,     /* pessimistic flood toward the goal     */
    EXPLORER_DONE
} ExplorerPhase;

typedef enum
{
    EXPLORER_STEP_PLAN,
	EXPLORER_STEP_WAIT_APPROACH,
    EXPLORER_STEP_WAIT_TURN,
    EXPLORER_STEP_WAIT_MOVE
} ExplorerStep;

typedef struct Explorer
{
    Navigator *nav;
    Mouse mouse;

    ExplorerPhase phase;
    ExplorerStep  step;

    Direction target_dir;
    bool run_speed_after;   /* after reaching goal, return + speed run */
} Explorer;

void Explorer_Init(Explorer *e, Navigator *nav);

/* Reset the belief maze and start exploring. */
void Explorer_Begin(Explorer *e, bool run_speed_after);

/*
 * Drive one step of the explore/solve loop. Non-blocking: call repeatedly
 * from the main loop. It issues navigator moves only while the navigator is
 * idle. feedback must be the current pose/yaw feedback.
 */
void Explorer_Tick(Explorer *e, const MotionFeedback *feedback);

ExplorerPhase Explorer_GetPhase(const Explorer *e);

#endif /* INC_EXPLORER_H_ */
