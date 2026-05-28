/*
 * robot_controller.c
 *
 *  Created on: May 28, 2026.
 *      Author: david
 */

#include "robot_controller.h"
#include <math.h>

static float _wrap_angle(float angle_rad)
{
    while (angle_rad >  (float)M_PI) angle_rad -= 2.0f * (float)M_PI;
    while (angle_rad <= -(float)M_PI) angle_rad += 2.0f * (float)M_PI;
    return angle_rad;
}

static float _clamp_float(float val, float min_val, float max_val)
{
	if (val < min_val) return min_val;
	if (val > max_val) return max_val;
	return val;
}

static void _clamp_wheel_speeds(float *omega_left_rad_s, float *omega_right_rad_s)
{
    float max_abs = fabsf(*omega_left_rad_s);
    if (fabsf(*omega_right_rad_s) > max_abs)
        max_abs = fabsf(*omega_right_rad_s);

    if (max_abs > ROBOT_MAX_WHEEL_SPEED_RAD_S)
    {
        float scale = ROBOT_MAX_WHEEL_SPEED_RAD_S / max_abs;
        *omega_left_rad_s  *= scale;
        *omega_right_rad_s *= scale;
    }
}

static void _body_vel_to_wheel_vel(float linear_x_m_s,
                                  float angular_z_rad_s,
                                  float *omega_left_rad_s,
                                  float *omega_right_rad_s)
{
    const float half_base = ROBOT_WHEEL_BASE_M / 2.0f;

    *omega_left_rad_s  = (linear_x_m_s - angular_z_rad_s * half_base)
                         / ROBOT_WHEEL_RADIUS_M;

    *omega_right_rad_s = (linear_x_m_s + angular_z_rad_s * half_base)
                         / ROBOT_WHEEL_RADIUS_M;
}

static void _wheel_vel_to_body_vel(float omega_left_rad_s,
                                  float omega_right_rad_s,
                                  float *linear_x_m_s,
                                  float *angular_z_rad_s)
{
    *linear_x_m_s    = ROBOT_WHEEL_RADIUS_M
                       * (omega_right_rad_s + omega_left_rad_s) / 2.0f;

    *angular_z_rad_s = ROBOT_WHEEL_RADIUS_M
                       * (omega_right_rad_s - omega_left_rad_s)
                       / ROBOT_WHEEL_BASE_M;
}


void RobotController_Init(RobotController *robot,
                          Motor       *motor_left,
                          Motor       *motor_right,
                          Encoder     *encoder_left,
                          Encoder     *encoder_right,
                          VelocityPI  *pi_left,
                          VelocityPI  *pi_right)
{
    if (robot == NULL)
        return;

    robot->motor_left    = motor_left;
    robot->motor_right   = motor_right;
    robot->encoder_left  = encoder_left;
    robot->encoder_right = encoder_right;
    robot->pi_left       = pi_left;
    robot->pi_right      = pi_right;

    robot->cmd_linear_x_m_s     = 0.0f;
    robot->cmd_angular_z_rad_s  = 0.0f;

    robot->setpoint_left_rad_s  = 0.0f;
    robot->setpoint_right_rad_s = 0.0f;

    robot->velocity_left_rad_s  = 0.0f;
    robot->velocity_right_rad_s = 0.0f;

    RobotController_ResetOdometry(robot);
}

void RobotController_SetCmdVel(RobotController *robot,
                               float linear_x_m_s,
                               float angular_z_rad_s)
{
    if (robot == NULL)
        return;

    linear_x_m_s    = _clamp_float(linear_x_m_s,
                                      -ROBOT_MAX_LINEAR_VEL_M_S,
                                       ROBOT_MAX_LINEAR_VEL_M_S);
	angular_z_rad_s = _clamp_float(angular_z_rad_s,
                                      -ROBOT_MAX_ANGULAR_VEL_RAD_S,
                                       ROBOT_MAX_ANGULAR_VEL_RAD_S);

    robot->cmd_linear_x_m_s    = linear_x_m_s;
    robot->cmd_angular_z_rad_s = angular_z_rad_s;

    _body_vel_to_wheel_vel(linear_x_m_s,
                          angular_z_rad_s,
                          &robot->setpoint_left_rad_s,
                          &robot->setpoint_right_rad_s);

    _clamp_wheel_speeds(&robot->setpoint_left_rad_s, &robot->setpoint_right_rad_s);

    VelocityPI_SetSetpoint(robot->pi_left,  robot->setpoint_left_rad_s);
    VelocityPI_SetSetpoint(robot->pi_right, robot->setpoint_right_rad_s);
}

void RobotController_Update(RobotController *robot, float dt_s)
{
    if (robot == NULL || dt_s <= 0.0f)
        return;

    Encoder_Update(robot->encoder_left,  dt_s);
    Encoder_Update(robot->encoder_right, dt_s);

    float velocity_left_rad_s_raw = Encoder_GetRawVelocityRadPerSecond(robot->encoder_left);
    float velocity_right_rad_s_raw = Encoder_GetRawVelocityRadPerSecond(robot->encoder_right);

    robot->velocity_left_rad_s  = Encoder_GetVelocityRadPerSecond(robot->encoder_left);
    robot->velocity_right_rad_s = Encoder_GetVelocityRadPerSecond(robot->encoder_right);

    float cmd_left  = VelocityPI_Update(robot->pi_left, velocity_left_rad_s_raw);
    float cmd_right = VelocityPI_Update(robot->pi_right, velocity_right_rad_s_raw);

    Motor_Set(robot->motor_left,  (int16_t)cmd_left);
    Motor_Set(robot->motor_right, (int16_t)cmd_right);

    float linear_vel_m_s;
    float angular_vel_rad_s;

    _wheel_vel_to_body_vel(robot->velocity_left_rad_s,
                          robot->velocity_right_rad_s,
                          &linear_vel_m_s,
                          &angular_vel_rad_s);

    float delta_theta = angular_vel_rad_s * dt_s;
    float theta_mid   = robot->pose_theta_rad + delta_theta / 2.0f;

    robot->pose_x_m      += linear_vel_m_s * cosf(theta_mid) * dt_s;
    robot->pose_y_m      += linear_vel_m_s * sinf(theta_mid) * dt_s;
    robot->pose_theta_rad = _wrap_angle(robot->pose_theta_rad + delta_theta);
}

void RobotController_ResetOdometry(RobotController *robot)
{
    if (robot == NULL)
        return;

    robot->pose_x_m       = 0.0f;
    robot->pose_y_m       = 0.0f;
    robot->pose_theta_rad = 0.0f;
}

void RobotController_Stop(RobotController *robot)
{
    if (robot == NULL)
        return;

    RobotController_SetCmdVel(robot, 0.0f, 0.0f);

    VelocityPI_Reset(robot->pi_left);
    VelocityPI_Reset(robot->pi_right);

    Motor_Brake(robot->motor_left);
    Motor_Brake(robot->motor_right);
}


float RobotController_GetPoseX(const RobotController *robot)
{
    return (robot != NULL) ? robot->pose_x_m : 0.0f;
}

float RobotController_GetPoseY(const RobotController *robot)
{
    return (robot != NULL) ? robot->pose_y_m : 0.0f;
}

float RobotController_GetPoseTheta(const RobotController *robot)
{
    return (robot != NULL) ? robot->pose_theta_rad : 0.0f;
}

float RobotController_GetLinearVelocity(const RobotController *robot)
{
    if (robot == NULL)
        return 0.0f;

    float linear_vel, angular_vel;
    _wheel_vel_to_body_vel(robot->velocity_left_rad_s,
                          robot->velocity_right_rad_s,
                          &linear_vel,
                          &angular_vel);
    return linear_vel;
}

float RobotController_GetAngularVelocity(const RobotController *robot)
{
    if (robot == NULL)
        return 0.0f;

    float linear_vel, angular_vel;
    _wheel_vel_to_body_vel(robot->velocity_left_rad_s,
                          robot->velocity_right_rad_s,
                          &linear_vel,
                          &angular_vel);
    return angular_vel;
}
