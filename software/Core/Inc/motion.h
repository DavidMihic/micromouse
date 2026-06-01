/*
 * motion.h
 *
 *  Created on: May 29, 2026
 *      Author: andrija
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include <stdbool.h>

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_STRAIGHT,
    MOTION_TURN,
    MOTION_DONE,
    MOTION_ABORTED
} MotionMode;

typedef struct MotionConfig
{
    float control_dt_s;

    float max_linear_speed_mps;
    float max_linear_accel_mps2;

    float max_angular_speed_radps;
    float max_angular_accel_radps2;

    float straight_max_angular_speed_radps;

    float distance_kp;
    float turn_kp;
    float heading_kp;
    float lateral_kp;

    float straight_distance_tolerance_m;
    float straight_heading_tolerance_rad;
    float turn_angle_tolerance_rad;

    float stop_linear_speed_mps;
    float stop_angular_speed_radps;

    unsigned int finish_settle_samples;

    bool use_lateral_odometry_correction;

} MotionConfig;

typedef struct MotionFeedback
{
    float x_m;
    float y_m;
    float yaw_rad;

    float linear_speed_mps;
    float angular_speed_radps;

} MotionFeedback;

typedef struct MotionCommand
{
    float linear_mps;
    float angular_radps;
    bool active;

} MotionCommand;

typedef struct MotionController
{
    MotionConfig cfg;

    MotionMode mode;

    float start_x_m;
    float start_y_m;
    float start_yaw_rad;

    float target_distance_m;
    float target_angle_rad;

    float cruise_speed_mps;
    float turn_speed_radps;

    float accumulated_turn_rad;
    float last_yaw_rad;

    float previous_linear_mps;
    float previous_angular_radps;

    float last_remaining;
    unsigned int settle_counter;

} MotionController;

void Motion_DefaultConfig(MotionConfig *cfg);
void Motion_Init(MotionController *motion, const MotionConfig *cfg);

bool Motion_StartStraight(MotionController *motion,
                          const MotionFeedback *feedback,
                          float distance_m,
                          float cruise_speed_mps);

bool Motion_StartTurn(MotionController *motion,
                      const MotionFeedback *feedback,
                      float angle_rad,
                      float max_angular_speed_radps);

MotionMode Motion_Update(MotionController *motion,
                         const MotionFeedback *feedback,
                         MotionCommand *command);

void Motion_Stop(MotionController *motion);
void Motion_ClearDone(MotionController *motion);

bool Motion_IsBusy(const MotionController *motion);
bool Motion_IsDone(const MotionController *motion);

MotionMode Motion_GetMode(const MotionController *motion);
float Motion_GetRemaining(const MotionController *motion);

void Motion_ForceDone(MotionController *motion);

#endif /* INC_MOTION_H_ */
