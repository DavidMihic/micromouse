/*
 * motion.c
 *
 *  Created on: May 29, 2026
 *      Author: andrija
 */
#include "motion.h"

#include <math.h>
#include <stddef.h>

#define MOTION_PI      3.14159265358979323846f
#define MOTION_TWO_PI  6.28318530717958647692f

static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float min_f(float a, float b)
{
    return (a < b) ? a : b;
}

static float clamp_f(float x, float min_value, float max_value)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}

static float safe_sqrt_f(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    return sqrtf(x);
}

static float wrap_pi(float angle_rad)
{
    while (angle_rad > MOTION_PI)
    {
        angle_rad -= MOTION_TWO_PI;
    }

    while (angle_rad < -MOTION_PI)
    {
        angle_rad += MOTION_TWO_PI;
    }

    return angle_rad;
}

static float rate_limit(float previous, float target, float max_delta)
{
    float delta = target - previous;

    if (max_delta <= 0.0f)
    {
        return target;
    }

    if (delta > max_delta)
    {
        return previous + max_delta;
    }

    if (delta < -max_delta)
    {
        return previous - max_delta;
    }

    return target;
}

static void zero_command(MotionCommand *command)
{
    if (command == NULL)
    {
        return;
    }

    command->linear_mps = 0.0f;
    command->angular_radps = 0.0f;
    command->active = false;
}

static bool motion_can_start(const MotionController *motion)
{
    if (motion == NULL)
    {
        return false;
    }

    return (motion->mode == MOTION_IDLE) ||
           (motion->mode == MOTION_DONE) ||
           (motion->mode == MOTION_ABORTED);
}

void Motion_DefaultConfig(MotionConfig *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->control_dt_s = 0.01f;                 // 100 Hz

    cfg->max_linear_speed_mps = 0.35f;
    cfg->max_linear_accel_mps2 = 1.5f;

    cfg->max_angular_speed_radps = 5.0f;
    cfg->max_angular_accel_radps2 = 25.0f;

    cfg->straight_max_angular_speed_radps = 2.5f;

    cfg->distance_kp = 8.0f;
    cfg->turn_kp = 10.0f;
    cfg->heading_kp = 6.0f;

    /*
     * Lateral odometry correction is useful only if your pose estimate is good.
     * Start disabled. Heading hold alone is usually enough for first tests.
     */
    cfg->lateral_kp = 0.0f;
    cfg->use_lateral_odometry_correction = false;

    cfg->straight_distance_tolerance_m = 0.003f;       // 3 mm
    cfg->straight_heading_tolerance_rad = 0.035f;      // about 2 deg
    cfg->turn_angle_tolerance_rad = 0.017f;            // about 1 deg

    cfg->stop_linear_speed_mps = 0.02f;
    cfg->stop_angular_speed_radps = 0.10f;

    cfg->finish_settle_samples = 3;
}

void Motion_Init(MotionController *motion, const MotionConfig *cfg)
{
    MotionConfig default_cfg;

    if (motion == NULL)
    {
        return;
    }

    if (cfg == NULL)
    {
        Motion_DefaultConfig(&default_cfg);
        motion->cfg = default_cfg;
    }
    else
    {
        motion->cfg = *cfg;
    }

    motion->mode = MOTION_IDLE;

    motion->start_x_m = 0.0f;
    motion->start_y_m = 0.0f;
    motion->start_yaw_rad = 0.0f;

    motion->target_distance_m = 0.0f;
    motion->target_angle_rad = 0.0f;

    motion->cruise_speed_mps = 0.0f;
    motion->turn_speed_radps = 0.0f;

    motion->accumulated_turn_rad = 0.0f;
    motion->last_yaw_rad = 0.0f;

    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = 0.0f;

    motion->last_remaining = 0.0f;
    motion->settle_counter = 0;
}

bool Motion_StartStraight(MotionController *motion,
                          const MotionFeedback *feedback,
                          float distance_m,
                          float cruise_speed_mps)
{
    if ((motion == NULL) || (feedback == NULL))
    {
        return false;
    }

    if (!motion_can_start(motion))
    {
        return false;
    }

    motion->mode = MOTION_STRAIGHT;

    motion->start_x_m = feedback->x_m;
    motion->start_y_m = feedback->y_m;
    motion->start_yaw_rad = feedback->yaw_rad;

    motion->target_distance_m = distance_m;
    motion->target_angle_rad = 0.0f;

    motion->cruise_speed_mps = abs_f(cruise_speed_mps);

    if (motion->cruise_speed_mps <= 0.0f)
    {
        motion->cruise_speed_mps = motion->cfg.max_linear_speed_mps;
    }

    motion->cruise_speed_mps = min_f(motion->cruise_speed_mps,
                                     motion->cfg.max_linear_speed_mps);

    motion->turn_speed_radps = 0.0f;

    motion->accumulated_turn_rad = 0.0f;
    motion->last_yaw_rad = feedback->yaw_rad;

    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = 0.0f;

    motion->last_remaining = distance_m;
    motion->settle_counter = 0;

    return true;
}

bool Motion_StartTurn(MotionController *motion,
                      const MotionFeedback *feedback,
                      float angle_rad,
                      float max_angular_speed_radps)
{
    if ((motion == NULL) || (feedback == NULL))
    {
        return false;
    }

    if (!motion_can_start(motion))
    {
        return false;
    }

    motion->mode = MOTION_TURN;

    motion->start_x_m = feedback->x_m;
    motion->start_y_m = feedback->y_m;
    motion->start_yaw_rad = feedback->yaw_rad;

    motion->target_distance_m = 0.0f;
    motion->target_angle_rad = angle_rad;

    motion->cruise_speed_mps = 0.0f;

    motion->turn_speed_radps = abs_f(max_angular_speed_radps);

    if (motion->turn_speed_radps <= 0.0f)
    {
        motion->turn_speed_radps = motion->cfg.max_angular_speed_radps;
    }

    motion->turn_speed_radps = min_f(motion->turn_speed_radps,
                                     motion->cfg.max_angular_speed_radps);

    motion->accumulated_turn_rad = 0.0f;
    motion->last_yaw_rad = feedback->yaw_rad;

    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = 0.0f;

    motion->last_remaining = angle_rad;
    motion->settle_counter = 0;

    return true;
}

static bool update_finish_counter(MotionController *motion,
                                  bool near_target,
                                  bool slow_enough)
{
    if (near_target && slow_enough)
    {
        if (motion->settle_counter < motion->cfg.finish_settle_samples)
        {
            motion->settle_counter++;
        }
    }
    else
    {
        motion->settle_counter = 0;
    }

    return motion->settle_counter >= motion->cfg.finish_settle_samples;
}

static void finish_motion(MotionController *motion, MotionCommand *command)
{
    motion->mode = MOTION_DONE;
    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = 0.0f;
    zero_command(command);
}

static void update_straight(MotionController *motion,
                            const MotionFeedback *feedback,
                            MotionCommand *command)
{
    const MotionConfig *cfg;

    float dx;
    float dy;
    float cos_yaw;
    float sin_yaw;

    float progress_m;
    float lateral_error_m;
    float remaining_m;

    float v_position;
    float v_stop_limit;
    float v_limit;
    float v_target;
    float v_cmd;

    float yaw_error_rad;
    float omega_target;
    float omega_cmd;
    float omega_limit;

    float max_dv;
    float max_domega;

    bool near_position;
    bool near_heading;
    bool slow_enough;
    bool finished;

    cfg = &motion->cfg;

    dx = feedback->x_m - motion->start_x_m;
    dy = feedback->y_m - motion->start_y_m;

    cos_yaw = cosf(motion->start_yaw_rad);
    sin_yaw = sinf(motion->start_yaw_rad);

    /*
     * Project odometry displacement onto the starting heading.
     */
    progress_m = cos_yaw * dx + sin_yaw * dy;

    /*
     * Positive lateral error means the robot is left of the desired line,
     * assuming standard x-forward, y-left robot/world convention.
     */
    lateral_error_m = -sin_yaw * dx + cos_yaw * dy;

    remaining_m = motion->target_distance_m - progress_m;
    motion->last_remaining = remaining_m;

    /*
     * Linear velocity command:
     * - proportional position controller
     * - limited by cruise speed
     * - limited by braking distance
     * - acceleration limited
     */
    v_position = cfg->distance_kp * remaining_m;

    v_stop_limit = safe_sqrt_f(2.0f *
                               cfg->max_linear_accel_mps2 *
                               abs_f(remaining_m));

    v_limit = min_f(motion->cruise_speed_mps, v_stop_limit);

    v_target = clamp_f(v_position, -v_limit, v_limit);

    if (abs_f(remaining_m) <= cfg->straight_distance_tolerance_m)
    {
        v_target = 0.0f;
    }

    max_dv = cfg->max_linear_accel_mps2 * cfg->control_dt_s;
    v_cmd = rate_limit(motion->previous_linear_mps, v_target, max_dv);

    /*
     * Angular velocity command:
     * - hold the heading that existed when the straight move started
     * - optional lateral odometry correction
     */
    yaw_error_rad = wrap_pi(motion->start_yaw_rad - feedback->yaw_rad);

    omega_target = cfg->heading_kp * yaw_error_rad;

    if (cfg->use_lateral_odometry_correction)
    {
        /*
         * Positive lateral error means robot is left of the desired line.
         * Command negative omega to steer right.
         */
        omega_target += -cfg->lateral_kp * lateral_error_m;
    }

    omega_limit = cfg->straight_max_angular_speed_radps;

    if (omega_limit <= 0.0f)
    {
        omega_limit = cfg->max_angular_speed_radps;
    }

    omega_target = clamp_f(omega_target, -omega_limit, omega_limit);

    max_domega = cfg->max_angular_accel_radps2 * cfg->control_dt_s;

    omega_cmd = rate_limit(motion->previous_angular_radps,
                           omega_target,
                           max_domega);

    motion->previous_linear_mps = v_cmd;
    motion->previous_angular_radps = omega_cmd;

    command->linear_mps = v_cmd;
    command->angular_radps = omega_cmd;
    command->active = true;

    near_position = abs_f(remaining_m) <= cfg->straight_distance_tolerance_m;
    near_heading = abs_f(yaw_error_rad) <= cfg->straight_heading_tolerance_rad;

    slow_enough =
        (abs_f(feedback->linear_speed_mps) <= cfg->stop_linear_speed_mps) &&
        (abs_f(feedback->angular_speed_radps) <= cfg->stop_angular_speed_radps);

    finished = update_finish_counter(motion,
                                     near_position && near_heading,
                                     slow_enough);

    if (finished)
    {
        finish_motion(motion, command);
    }
}

static void update_turn(MotionController *motion,
                        const MotionFeedback *feedback,
                        MotionCommand *command)
{
    const MotionConfig *cfg;

    float delta_yaw;
    float remaining_rad;

    float omega_position;
    float omega_stop_limit;
    float omega_limit;
    float omega_target;
    float omega_cmd;

    float max_domega;

    bool near_angle;
    bool slow_enough;
    bool finished;

    cfg = &motion->cfg;

    /*
     * Use accumulated unwrapped yaw change so that turns across +/-pi work.
     */
    delta_yaw = wrap_pi(feedback->yaw_rad - motion->last_yaw_rad);
    motion->accumulated_turn_rad += delta_yaw;
    motion->last_yaw_rad = feedback->yaw_rad;

    remaining_rad = motion->target_angle_rad - motion->accumulated_turn_rad;
    motion->last_remaining = remaining_rad;

    /*
     * Angular velocity command:
     * - proportional angle controller
     * - limited by max turn speed
     * - limited by braking angle
     * - acceleration limited
     */
    omega_position = cfg->turn_kp * remaining_rad;

    omega_stop_limit = safe_sqrt_f(2.0f *
                                   cfg->max_angular_accel_radps2 *
                                   abs_f(remaining_rad));

    omega_limit = min_f(motion->turn_speed_radps, omega_stop_limit);

    omega_target = clamp_f(omega_position, -omega_limit, omega_limit);

    if (abs_f(remaining_rad) <= cfg->turn_angle_tolerance_rad)
    {
        omega_target = 0.0f;
    }

    max_domega = cfg->max_angular_accel_radps2 * cfg->control_dt_s;

    omega_cmd = rate_limit(motion->previous_angular_radps,
                           omega_target,
                           max_domega);

    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = omega_cmd;

    command->linear_mps = 0.0f;
    command->angular_radps = omega_cmd;
    command->active = true;

    near_angle = abs_f(remaining_rad) <= cfg->turn_angle_tolerance_rad;

    slow_enough =
        (abs_f(feedback->linear_speed_mps) <= cfg->stop_linear_speed_mps) &&
        (abs_f(feedback->angular_speed_radps) <= cfg->stop_angular_speed_radps);

    finished = update_finish_counter(motion, near_angle, slow_enough);

    if (finished)
    {
        finish_motion(motion, command);
    }
}

MotionMode Motion_Update(MotionController *motion,
                         const MotionFeedback *feedback,
                         MotionCommand *command)
{
    if ((motion == NULL) || (feedback == NULL) || (command == NULL))
    {
        return MOTION_ABORTED;
    }

    if (motion->cfg.control_dt_s <= 0.0f)
    {
        motion->cfg.control_dt_s = 0.01f;
    }

    switch (motion->mode)
    {
        case MOTION_STRAIGHT:
        {
            update_straight(motion, feedback, command);
            break;
        }

        case MOTION_TURN:
        {
            update_turn(motion, feedback, command);
            break;
        }

        case MOTION_IDLE:
        case MOTION_DONE:
        case MOTION_ABORTED:
        default:
        {
            zero_command(command);
            break;
        }
    }

    return motion->mode;
}

void Motion_Stop(MotionController *motion)
{
    if (motion == NULL)
    {
        return;
    }

    motion->mode = MOTION_ABORTED;
    motion->previous_linear_mps = 0.0f;
    motion->previous_angular_radps = 0.0f;
    motion->settle_counter = 0;
}

void Motion_ClearDone(MotionController *motion)
{
    if (motion == NULL)
    {
        return;
    }

    if ((motion->mode == MOTION_DONE) || (motion->mode == MOTION_ABORTED))
    {
        motion->mode = MOTION_IDLE;
    }
}

bool Motion_IsBusy(const MotionController *motion)
{
    if (motion == NULL)
    {
        return false;
    }

    return (motion->mode == MOTION_STRAIGHT) ||
           (motion->mode == MOTION_TURN);
}

bool Motion_IsDone(const MotionController *motion)
{
    if (motion == NULL)
    {
        return false;
    }

    return motion->mode == MOTION_DONE;
}

MotionMode Motion_GetMode(const MotionController *motion)
{
    if (motion == NULL)
    {
        return MOTION_ABORTED;
    }

    return motion->mode;
}

float Motion_GetRemaining(const MotionController *motion)
{
    if (motion == NULL)
    {
        return 0.0f;
    }

    return motion->last_remaining;
}
