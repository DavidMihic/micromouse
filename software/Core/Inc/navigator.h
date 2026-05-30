/*
 * navigator.h
 *
 *  Created on: May 30, 2026.
 *      Author: david
 */

#ifndef INC_NAVIGATOR_H_
#define INC_NAVIGATOR_H_

#include <stdint.h>
#include <stdbool.h>

#include "motion.h"
#include "robot_controller.h"

/*
 * Cardinal heading. Stored clockwise so a right turn is +1 and a left turn
 * is -1 (both modulo 4). The mapping to maze axes is fixed in navigator.c.
 */
typedef enum
{
    HEADING_NORTH = 0,
    HEADING_EAST  = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST  = 3
} Heading;

typedef struct NavigatorConfig
{
    /* Geometry */
    float cell_size_m;

    /* Default speeds for whole-cell moves and turns */
    float cruise_speed_mps;
    float turn_speed_radps;

    /*
     * Sensor role mapping. Indices into the IR signal[] array, where
     * signal[i] = IR_REC_(i+1). Physical layout (symmetric):
     *   sensor 1 (idx 0) = right side      (90 deg)
     *   sensor 2 (idx 1) = right diagonal  (45 deg)
     *   sensor 3 (idx 2) = right front
     *   sensor 4 (idx 3) = left front
     *   sensor 5 (idx 4) = left diagonal   (45 deg)
     *   sensor 6 (idx 5) = left side       (90 deg)
     */
    uint8_t side_left_index;
    uint8_t side_right_index;
    uint8_t diag_left_index;
    uint8_t diag_right_index;
    uint8_t front_left_index;
    uint8_t front_right_index;

    /* A wall is present on a side when its sensor exceeds this threshold. */
    int32_t wall_present_threshold;

    /* Front wall present when both front sensors exceed this threshold. */
    int32_t front_wall_threshold;

    /*
     * Diagonal sensor reading when the robot is centred in the corridor.
     * Used for single-wall following. Must be calibrated. For a corridor with
     * walls on both sides the reference cancels out and is not needed.
     */
    int32_t diag_centered_reading;

    /*
     * Centering controller: converts a (right_diag - left_diag) error into an
     * angular velocity bias [rad/s], clamped to max_centering_radps.
     */
    float centering_kp;
    float max_centering_radps;

} NavigatorConfig;

typedef struct Navigator
{
    NavigatorConfig cfg;

    MotionController *motion;
    RobotController  *robot;

    Heading heading;
    int32_t cell_x;
    int32_t cell_y;

    /* True while side-wall centering should be applied (set during straights). */
    bool centering_enabled;

    /* Heading delta committed when the current turn finishes (-1, +1, +2). */
    int8_t pending_heading_step;

    /* Number of cells to commit when the current straight finishes. */
	int32_t pending_cells;

    /* Edge detection for motion completion. */
    bool was_busy;

    int32_t last_diag_left;
	int32_t last_diag_right;
	float   last_correction;

} Navigator;

void Navigator_DefaultConfig(NavigatorConfig *cfg);

void Navigator_Init(Navigator *nav,
                    MotionController *motion,
                    RobotController *robot,
                    const NavigatorConfig *cfg,
                    Heading start_heading);

/*
 * High-level moves. Each starts a motion and returns immediately.
 * Poll Navigator_IsBusy() / Navigator_IsIdle() to wait for completion.
 * feedback must be the same pose/yaw feedback fed to Motion_Update.
 */
bool Navigator_MoveForward(Navigator *nav, const MotionFeedback *feedback);
bool Navigator_MoveForwardCells(Navigator *nav, const MotionFeedback *feedback, int32_t cells);
bool Navigator_TurnLeft(Navigator *nav, const MotionFeedback *feedback);
bool Navigator_TurnRight(Navigator *nav, const MotionFeedback *feedback);
bool Navigator_TurnAround(Navigator *nav, const MotionFeedback *feedback);

/*
 * Per-control-tick hook. Call this from the control ISR right AFTER
 * Motion_Update() and BEFORE pushing command to the robot. It:
 *   - applies side-wall centering to command->angular_radps during straights
 *   - detects motion completion and commits the heading / cell update
 */
void Navigator_ControlTick(Navigator *nav,
                           const MotionFeedback *feedback,
                           MotionCommand *command);

bool Navigator_IsBusy(const Navigator *nav);
bool Navigator_IsIdle(const Navigator *nav);

/* Wall queries based on the latest IR frame. */
bool Navigator_WallLeft(const Navigator *nav);
bool Navigator_WallRight(const Navigator *nav);
bool Navigator_WallFront(const Navigator *nav);

Heading Navigator_GetHeading(const Navigator *nav);
int32_t Navigator_GetCellX(const Navigator *nav);
int32_t Navigator_GetCellY(const Navigator *nav);

int32_t Navigator_GetLastDiagLeft(const Navigator *nav);
int32_t Navigator_GetLastDiagRight(const Navigator *nav);
float   Navigator_GetLastCorrection(const Navigator *nav);
bool    Navigator_CenteringActive(const Navigator *nav);
#endif /* INC_NAVIGATOR_H_ */
