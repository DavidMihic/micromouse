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

#define PARAM_CELL_DIM 0.16f // metres
#define PARAM_MAZE_DIM 18u

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
#define PARAM_MOTION_MAX_LINEAR_ACCEL_MPS2   4.0f

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

#endif /* INC_PARAMS_H_ */
