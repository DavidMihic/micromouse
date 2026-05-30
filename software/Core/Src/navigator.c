/*
 * navigator.c
 *
 *  Created on: May 30, 2026.
 *      Author: david
 */

#include "navigator.h"

#include "params.h"
#include "ir_sensors.h"

#include <stddef.h>
#include <math.h>

#define NAV_PI       3.14159265358979323846f
#define NAV_HALF_PI  1.57079632679489661923f

static float _clamp_f(float val, float min_value, float max_value)
{
    if (val < min_value)
        return min_value;
    if (val > max_value)
        return max_value;
    return val;
}

static int32_t _ir_signal(uint8_t index)
{
    const int32_t *signal = IR_Sensors_GetSignal();

    if (signal == NULL)
        return 0;

    return signal[index];
}

/* Advance the integer cell coordinates by one cell along the current heading. */
static void _step_cell(Navigator *nav)
{
    switch (nav->heading)
    {
        case HEADING_NORTH: nav->cell_y += 1; break;
        case HEADING_EAST:  nav->cell_x += 1; break;
        case HEADING_SOUTH: nav->cell_y -= 1; break;
        case HEADING_WEST:  nav->cell_x -= 1; break;
        default: break;
    }
}

/* Apply a heading step (-1 left, +1 right, +2 around), wrapped to 0..3. */
static void _apply_heading_step(Navigator *nav, int8_t step)
{
    int32_t h = (int32_t)nav->heading + step;

    h %= 4;
    if (h < 0)
        h += 4;

    nav->heading = (Heading)h;
}

/*
 * Side-wall centering using the 45 deg diagonal sensors. Produces an angular
 * velocity bias [rad/s].
 *
 * Convention: positive angular velocity is CCW (a left turn). Too close to the
 * RIGHT wall -> steer LEFT (positive omega); too close to the LEFT wall ->
 * steer RIGHT (negative omega).
 *
 * Diagonals are used instead of the 90 deg side sensors because near the cell
 * centre the side sensors are close to saturation and barely change with small
 * lateral offsets, while the diagonals stay sensitive.
 *
 * Both diagonals see a wall -> balance right vs left (reference cancels).
 * One diagonal sees a wall  -> hold the calibrated diag_centered_reading.
 * Neither                    -> no correction (gyro heading-hold keeps course).
 */
static float _centering_correction(const Navigator *nav)
{
    const NavigatorConfig *cfg = &nav->cfg;

    int32_t right = _ir_signal(cfg->diag_right_index);
    int32_t left  = _ir_signal(cfg->diag_left_index);

    bool right_present = right > cfg->wall_present_threshold;
    bool left_present  = left  > cfg->wall_present_threshold;

    if (!left_present && !right_present)
        return 0.0f;

    float right_error = right_present ? (float)(right - cfg->diag_centered_reading) : 0.0f;
    float left_error  = left_present  ? (float)(left  - cfg->diag_centered_reading) : 0.0f;

    /*
     * right_error > 0 -> too close on the right -> steer left (positive omega).
     * left_error  > 0 -> too close on the left  -> steer right (negative omega).
     */
    float correction = cfg->centering_kp * (right_error - left_error);

    return _clamp_f(correction,
                    -cfg->max_centering_radps,
                     cfg->max_centering_radps);
}

void Navigator_DefaultConfig(NavigatorConfig *cfg)
{
    if (cfg == NULL)
        return;

    cfg->cell_size_m      = PARAM_CELL_DIM;
    cfg->cruise_speed_mps = 0.30f;
    cfg->turn_speed_radps = 3.0f;

    cfg->side_right_index  = 0;   /* IR_REC_1, right side 90 deg     */
    cfg->diag_right_index  = 1;   /* IR_REC_2, right diagonal 45 deg */
    cfg->front_right_index = 2;   /* IR_REC_3, right front           */
    cfg->front_left_index  = 3;   /* IR_REC_4, left front            */
    cfg->diag_left_index   = 4;   /* IR_REC_5, left diagonal 45 deg  */
    cfg->side_left_index   = 5;   /* IR_REC_6, left side 90 deg      */

    cfg->wall_present_threshold = IR_SEN_DIGTAL_TRESHOLD;
    cfg->front_wall_threshold   = IR_SEN_DIGTAL_TRESHOLD;

    /* Diagonal reading when centred (measured on the robot). */
    cfg->diag_centered_reading = 3200;

    cfg->centering_kp        = 0.002f;
    cfg->max_centering_radps = 3.0f;
}

void Navigator_Init(Navigator *nav,
                    MotionController *motion,
                    RobotController *robot,
                    const NavigatorConfig *cfg,
                    Heading start_heading)
{
    NavigatorConfig default_cfg;

    if (nav == NULL)
        return;

    if (cfg == NULL)
    {
        Navigator_DefaultConfig(&default_cfg);
        nav->cfg = default_cfg;
    }
    else
        nav->cfg = *cfg;

    nav->motion = motion;
    nav->robot  = robot;

    nav->heading = start_heading;
    nav->cell_x  = 0;
    nav->cell_y  = 0;

    nav->centering_enabled    = true;
    nav->pending_heading_step = 0;
    nav->was_busy             = false;

    nav->last_diag_left  = 0;
    nav->last_diag_right = 0;
    nav->last_correction = 0.0f;
}

bool Navigator_MoveForward(Navigator *nav, const MotionFeedback *feedback)
{
	return Navigator_MoveForwardCells(nav, feedback, 1);
}

bool Navigator_MoveForwardCells(Navigator *nav, const MotionFeedback *feedback, int32_t cells)
{
    if ((nav == NULL) || (feedback == NULL))
        return false;

    if (cells < 1)
        return false;

    if (Navigator_IsBusy(nav))
        return false;

    if (!Motion_StartStraight(nav->motion,
                              feedback,
                              nav->cfg.cell_size_m * (float)cells,
                              nav->cfg.cruise_speed_mps))
        return false;

    nav->pending_heading_step = 0;
    nav->pending_cells        = cells;
    nav->centering_enabled    = true;
    nav->was_busy             = true;

    return true;
}

static bool _start_turn(Navigator *nav,
                        const MotionFeedback *feedback,
                        float angle_rad,
                        int8_t heading_step)
{
    if ((nav == NULL) || (feedback == NULL))
        return false;

    if (Navigator_IsBusy(nav))
        return false;

    if (!Motion_StartTurn(nav->motion,
                          feedback,
                          angle_rad,
                          nav->cfg.turn_speed_radps))
        return false;

    nav->pending_heading_step = heading_step;
    nav->centering_enabled    = false;   /* no side walls relevant while turning */
    nav->was_busy             = true;

    return true;
}

bool Navigator_TurnLeft(Navigator *nav, const MotionFeedback *feedback)
{
    /* CCW is positive; left turn = +90 deg = heading step -1. */
    return _start_turn(nav, feedback, +NAV_HALF_PI, -1);
}

bool Navigator_TurnRight(Navigator *nav, const MotionFeedback *feedback)
{
    /* CW = negative angle; right turn = heading step +1. */
    return _start_turn(nav, feedback, -NAV_HALF_PI, +1);
}

bool Navigator_TurnAround(Navigator *nav, const MotionFeedback *feedback)
{
    return _start_turn(nav, feedback, NAV_PI, +2);
}

void Navigator_ControlTick(Navigator *nav,
                           const MotionFeedback *feedback,
                           MotionCommand *command)
{
    (void)feedback;

    if ((nav == NULL) || (command == NULL))
        return;

    /* Always read and store, even when idle, so it can be inspected. */
    nav->last_diag_right = _ir_signal(nav->cfg.diag_right_index);
    nav->last_diag_left  = _ir_signal(nav->cfg.diag_left_index);
    nav->last_correction = _centering_correction(nav);

    /* Apply centering only while actively driving a straight. */
    if (nav->centering_enabled &&
        command->active &&
        (Motion_GetMode(nav->motion) == MOTION_STRAIGHT))
    {
        command->angular_radps += nav->last_correction;
    }

    /* Detect the busy -> not-busy edge to finalise the move. */
    bool busy_now = Motion_IsBusy(nav->motion);

    if (nav->was_busy && !busy_now)
    {
        if (Motion_IsDone(nav->motion))
        {
            if (nav->pending_heading_step != 0)
                _apply_heading_step(nav, nav->pending_heading_step);
            else
                _step_cell(nav);
        }

        nav->pending_heading_step = 0;
        nav->centering_enabled    = false;

        /* Clear the DONE/ABORTED latch so the next move can start. */
        Motion_ClearDone(nav->motion);
    }

    nav->was_busy = busy_now;
}

bool Navigator_IsBusy(const Navigator *nav)
{
    if (nav == NULL)
        return false;

    return Motion_IsBusy(nav->motion);
}

bool Navigator_IsIdle(const Navigator *nav)
{
    return !Navigator_IsBusy(nav);
}

bool Navigator_WallLeft(const Navigator *nav)
{
    if (nav == NULL)
        return false;

    return _ir_signal(nav->cfg.side_left_index) > nav->cfg.wall_present_threshold;
}

bool Navigator_WallRight(const Navigator *nav)
{
    if (nav == NULL)
        return false;

    return _ir_signal(nav->cfg.side_right_index) > nav->cfg.wall_present_threshold;
}

bool Navigator_WallFront(const Navigator *nav)
{
    if (nav == NULL)
        return false;

    bool fl = _ir_signal(nav->cfg.front_left_index)  > nav->cfg.front_wall_threshold;
    bool fr = _ir_signal(nav->cfg.front_right_index) > nav->cfg.front_wall_threshold;

    return fl && fr;
}

Heading Navigator_GetHeading(const Navigator *nav)
{
    return (nav != NULL) ? nav->heading : HEADING_NORTH;
}

int32_t Navigator_GetCellX(const Navigator *nav)
{
    return (nav != NULL) ? nav->cell_x : 0;
}

int32_t Navigator_GetCellY(const Navigator *nav)
{
    return (nav != NULL) ? nav->cell_y : 0;
}

int32_t Navigator_GetLastDiagLeft(const Navigator *nav)
{
    return (nav != NULL) ? nav->last_diag_left : 0;
}

int32_t Navigator_GetLastDiagRight(const Navigator *nav)
{
    return (nav != NULL) ? nav->last_diag_right : 0;
}

float Navigator_GetLastCorrection(const Navigator *nav)
{
    return (nav != NULL) ? nav->last_correction : 0.0f;
}

bool Navigator_CenteringActive(const Navigator *nav)
{
    if (nav == NULL)
        return false;

    return nav->centering_enabled &&
           (Motion_GetMode(nav->motion) == MOTION_STRAIGHT);
}
