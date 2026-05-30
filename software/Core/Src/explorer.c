/*
 * explorer.c
 *
 *  Created on: May 30, 2026.
 *      Author: david
 */

#include "explorer.h"

#include "params.h"
#include "belief_maze.h"
#include "floodfill.h"

#include <stddef.h>

/* Sense the three forward-facing walls and write them into the belief maze. */
static void _sense(Explorer *e)
{
    Navigator *n = e->nav;

    uint8_t row = (uint8_t)Navigator_GetCellY(n);
    uint8_t col = (uint8_t)Navigator_GetCellX(n);
    int     h   = (int)Navigator_GetHeading(n);

    Direction front = (Direction)(h);
    Direction left  = (Direction)((h + 3) & 3);   /* CCW */
    Direction right = (Direction)((h + 1) & 3);   /* CW  */

    /*
     * Set both present and absent so cells become fully "known" for the
     * pessimistic speed-run. The cell we came from is already known open
     * from when its front wall was sensed, so the back wall is not touched.
     */
    belief_maze_set_wall(row, col, front, Navigator_WallFront(n));
    belief_maze_set_wall(row, col, left,  Navigator_WallLeft(n));
    belief_maze_set_wall(row, col, right, Navigator_WallRight(n));
}

static void _sync_mouse(Explorer *e)
{
    e->mouse.row     = (uint8_t)Navigator_GetCellY(e->nav);
    e->mouse.col     = (uint8_t)Navigator_GetCellX(e->nav);
    e->mouse.heading = (Direction)Navigator_GetHeading(e->nav);
}

/* Issue an in-place pivot toward target_dir given the current heading. */
static void _issue_pivot(Explorer *e, const MotionFeedback *feedback)
{
    int diff = ((int)e->target_dir - (int)Navigator_GetHeading(e->nav) + 4) & 3;

    switch (diff)
    {
        case 1: Navigator_TurnRight(e->nav, feedback);  break;
        case 3: Navigator_TurnLeft(e->nav, feedback);   break;
        default: Navigator_TurnAround(e->nav, feedback); break;  /* 2 */
    }
    e->step = EXPLORER_STEP_WAIT_TURN;
}

/*
 * Decide how to reach target_dir from the current heading.
 *  - straight ahead: drive one full cell.
 *  - turn needed: first nudge forward to the cell centre (PARAM_NAV_TURN_APPROACH_M),
 *    then pivot, then drive into the next cell.
 */
static void _start_toward(Explorer *e, const MotionFeedback *feedback, Direction dir)
{
    e->target_dir = dir;

    int diff = ((int)dir - (int)Navigator_GetHeading(e->nav) + 4) & 3;

    if (diff == 0)
    {
        Navigator_MoveForward(e->nav, feedback);
        e->step = EXPLORER_STEP_WAIT_MOVE;
        return;
    }
    if (PARAM_NAV_TURN_APPROACH_M > 0.0f)
	{
		Navigator_MoveDistance(e->nav, feedback, PARAM_NAV_TURN_APPROACH_M);
		e->step = EXPLORER_STEP_WAIT_APPROACH;
	}
	else
	{
		_issue_pivot(e, feedback);
	}
}

void Explorer_Init(Explorer *e, Navigator *nav)
{
    if (e == NULL)
        return;

    e->nav   = nav;
    e->phase = EXPLORER_IDLE;
    e->step  = EXPLORER_STEP_PLAN;
    e->target_dir = DIR_NORTH;
    e->run_speed_after = false;

    e->mouse.row = 0;
    e->mouse.col = 0;
    e->mouse.heading = DIR_NORTH;
}

void Explorer_Begin(Explorer *e, bool run_speed_after)
{
    if (e == NULL)
        return;

    belief_maze_init();

    e->phase = EXPLORER_EXPLORE;
    e->step  = EXPLORER_STEP_PLAN;
    e->run_speed_after = run_speed_after;
}

void Explorer_Tick(Explorer *e, const MotionFeedback *feedback)
{
    if ((e == NULL) || (feedback == NULL))
        return;

    if ((e->phase == EXPLORER_IDLE) || (e->phase == EXPLORER_DONE))
        return;

    /* Wait for the current navigator move/turn to finish. */
    if (Navigator_IsBusy(e->nav))
        return;

    switch (e->step)
    {
    	case EXPLORER_STEP_WAIT_APPROACH:
			/* Reached the cell centre; now pivot toward the target. */
			_issue_pivot(e, feedback);
			return;

        case EXPLORER_STEP_WAIT_TURN:
            /* Oriented; now drive forward into the next cell. */
            Navigator_MoveForward(e->nav, feedback);
            e->step = EXPLORER_STEP_WAIT_MOVE;
            return;

        case EXPLORER_STEP_WAIT_MOVE:
            /* Arrived in the new cell; navigator pose is updated. Plan next. */
            e->step = EXPLORER_STEP_PLAN;
            return;

        case EXPLORER_STEP_PLAN:
        default:
            break;
    }

    /* PLAN: sense the current cell, then decide and issue the next move. */
    _sense(e);
    _sync_mouse(e);

    /* Phase transitions on arrival at goal / start. */
    if ((e->phase == EXPLORER_EXPLORE) && planner_at_goal(&e->mouse))
    {
        if (!e->run_speed_after)
        {
            e->phase = EXPLORER_DONE;
            return;
        }
        e->phase = EXPLORER_RETURN;
    }
    else if ((e->phase == EXPLORER_RETURN) && planner_at_start(&e->mouse))
    {
        e->phase = EXPLORER_SPEED;
    }
    else if ((e->phase == EXPLORER_SPEED) && planner_at_goal(&e->mouse))
    {
        e->phase = EXPLORER_DONE;
        return;
    }

    Direction dir;

    switch (e->phase)
    {
        case EXPLORER_EXPLORE:
            floodfill_compute_on_belief();
            dir = planner_next_move_belief(&e->mouse);
            break;
        case EXPLORER_RETURN:
            floodfill_compute_return_on_belief();
            dir = planner_next_move_belief(&e->mouse);
            break;
        case EXPLORER_SPEED:
        default:
            floodfill_compute_on_belief_pessimistic();
            dir = planner_next_move_belief_pessimistic(&e->mouse);
            break;
    }

    _start_toward(e, feedback, dir);
}

ExplorerPhase Explorer_GetPhase(const Explorer *e)
{
    return (e != NULL) ? e->phase : EXPLORER_IDLE;
}
