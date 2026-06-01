/*
 * params.h
 *
 *  Central tuning parameters for the Micromouse.
 *
 *  Created on: May 30, 2026.
 *      Author: david
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Wheel velocity PI                                                         */
/* ------------------------------------------------------------------------- */
#define PARAM_PI_KP            11.5f
#define PARAM_PI_KI            300.0f
#define PARAM_PI_MAX_PWM_OUT   1000.0f   /* matches MOTOR_PWM_MAX */

/* ------------------------------------------------------------------------- */
/* Encoders                                                                  */
/* ------------------------------------------------------------------------- */
/* Counts per WHEEL revolution (already includes x4 quadrature + gearbox). */
#define PARAM_ENC_TICKS_PER_REV        360.0f
#define PARAM_ENC_VEL_FILTER_TAU_S     0.01f
#define PARAM_ENC_DIR_LEFT             (-1)
#define PARAM_ENC_DIR_RIGHT            (1)

/* ------------------------------------------------------------------------- */
/* Motor wiring direction                                                    */
/* ------------------------------------------------------------------------- */
#define PARAM_MOTOR_DIR_LEFT           (-1)
#define PARAM_MOTOR_DIR_RIGHT          (1)

/* ------------------------------------------------------------------------- */
/* Motion profile                                                            */
/* ------------------------------------------------------------------------- */
#define PARAM_MOTION_MAX_LINEAR_SPEED_MPS    2.0f
#define PARAM_MOTION_MAX_LINEAR_ACCEL_MPS2   4.5f

#define PARAM_MOTION_MAX_ANGULAR_SPEED_RADPS  20.0f
#define PARAM_MOTION_MAX_ANGULAR_ACCEL_RADPS2 50.0f

#define PARAM_MOTION_DISTANCE_KP        4.75f
#define PARAM_MOTION_HEADING_KP         6.0f
#define PARAM_MOTION_TURN_KP            9.0f

#define PARAM_MOTION_USE_LATERAL_CORR   false
#define PARAM_MOTION_LATERAL_KP         0.0f

#define PARAM_MOTION_DIST_TOL_M         0.003f   /* 3 mm  */
#define PARAM_MOTION_HEADING_TOL_RAD    0.035f   /* ~2 deg */
#define PARAM_MOTION_TURN_TOL_RAD       0.017f   /* ~1 deg */
#define PARAM_MOTION_STOP_LINEAR_MPS    0.02f
#define PARAM_MOTION_STOP_ANGULAR_RADPS 0.10f
#define PARAM_MOTION_SETTLE_SAMPLES     3u

/* ------------------------------------------------------------------------- */
/* IMU                                                                       */
/* ------------------------------------------------------------------------- */
#define PARAM_IMU_CALIB_SAMPLES         500u
#define PARAM_IMU_CALIB_SETTLE_MS       1000u

/* ------------------------------------------------------------------------- */
/* IR sensors (Keep max pulse short so the LEDs cannot remain on for long.)  */
/* ------------------------------------------------------------------------- */
#define PARAM_IR_PULSE_SETTLE_US        100u
#define PARAM_IR_MAX_PULSE_US           200u
#define PARAM_IR_ADC_TIMEOUT_US         1000u

/* ------------------------------------------------------------------------- */
/* Maze solver                                                               */
/* ------------------------------------------------------------------------- */
#define PARAM_MAZE_CELL_DISTANCE_M       0.180f

/* Exploration stays slow/reliable. Return and fast run use the same
 * cell-by-cell mechanics, just with higher speed limits.
 */
#define PARAM_MAZE_CRUISE_SPEED_MPS        2.0f
#define PARAM_MAZE_TURN_SPEED_RADPS        10.0f

#define PARAM_MAZE_RETURN_CRUISE_SPEED_MPS 2.00f
#define PARAM_MAZE_RETURN_TURN_SPEED_RADPS 10.0f

#define PARAM_MAZE_FAST_CRUISE_SPEED_MPS   2.0f
#define PARAM_MAZE_FAST_TURN_SPEED_RADPS   10.0f

/*
 * Tune these tonight.
 * They use IR_Sensors_GetSignal(), not ADC raw directly.
 */
#define PARAM_MAZE_LEFT_WALL_TH          2500
#define PARAM_MAZE_FRONT_WALL_TH         2500
#define PARAM_MAZE_RIGHT_WALL_TH         2500

/* ------------------------------------------------------------------------- */
/* IR sensor physical mapping                                                */
/* ------------------------------------------------------------------------- */
/* Current assumed order from IR_Sensors_GetSignal():
 *   0 = right horizontal
 *   1 = right diagonal
 *   2 = front right
 *   3 = front left
 *   4 = left diagonal
 *   5 = left horizontal
 */
#define PARAM_IR_IDX_RIGHT_H             0u
#define PARAM_IR_IDX_RIGHT_DIAG          1u
#define PARAM_IR_IDX_FRONT_R             2u
#define PARAM_IR_IDX_FRONT_L             3u
#define PARAM_IR_IDX_LEFT_DIAG           4u
#define PARAM_IR_IDX_LEFT_H              5u

/* ------------------------------------------------------------------------- */
/* Simple wall assist during cell-to-cell straight moves                     */
/* ------------------------------------------------------------------------- */
#define PARAM_CENTERING_ENABLE           true

/* Diagonal sensor must be above this to count as seeing a side wall. */
#define PARAM_CENTERING_DIAG_WALL_TH     1500

/* Ignore small left-right diagonal differences. */
#define PARAM_CENTERING_DEADBAND_COUNTS  60

/* omega_correction = KP * (right_diag - left_diag).
 * If centering drives the robot the wrong way, change this to -1.0f.
 */
#define PARAM_CENTERING_SIGN             1.0f
#define PARAM_CENTERING_KP_RADPS_PER_COUNT 0.002f
#define PARAM_CENTERING_MAX_OMEGA_RADPS  1.0f

/* Clamp total straight-line angular command after adding IR correction. */
#define PARAM_CENTERING_TOTAL_OMEGA_LIMIT_RADPS 2.0f

/* 0..1 first-order smoothing for IR readings; larger = faster/noisier. */
#define PARAM_CENTERING_FILTER_ALPHA     0.75f

/* If no new IR frame for this many 100 Hz samples, do not use old correction. */
#define PARAM_CENTERING_STALE_SAMPLES    15u

/* ------------------------------------------------------------------------- */
/* Front wall anti-collision during straight moves                           */
/* ------------------------------------------------------------------------- */
#define PARAM_FRONT_AC_ENABLE            true

/* If front sensor exceeds this close to the end of a cell, finish the move
 * early and let the maze solver continue from the next cell.
 */
#define PARAM_FRONT_AC_STOP_TH           3920
#define PARAM_FRONT_AC_FINISH_REMAINING_M 0.040f

/* If front sensor exceeds this while still far from the target, abort and stop. */
#define PARAM_FRONT_AC_ABORT_TH          4095

#define PARAM_CENTERING_DISABLE_END_REMAINING_M 0.020f

/* If front sensors are already high, diagonal sensors are probably seeing a corner.
 * Keep this lower than your front stop threshold.
 */
#define PARAM_CENTERING_FRONT_DISABLE_TH 2400

/* Diagonal readings above this are probably corner/cross-wall reflections,
 * not normal centered corridor readings.
 * Tune this from printf data.
 */
#define PARAM_CENTERING_DIAG_MAX_VALID_TH 4095

/* One-wall fallback targets.
 * Put robot centered in a straight corridor and print diagonal values.
 * Set these close to the normal centered readings.
 */
#define PARAM_CENTERING_LEFT_DIAG_TARGET   2300.0f
#define PARAM_CENTERING_RIGHT_DIAG_TARGET  2300.0f


#endif /* INC_PARAMS_H_ */
