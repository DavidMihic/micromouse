/*
 * robot_controller.h
 *
 *  Created on: May 28, 2026.
 *      Author: david
 */

#ifndef INC_ROBOT_CONTROLLER_H_
#define INC_ROBOT_CONTROLLER_H_

#include "encoder.h"
#include "motor.h"
#include "velocity_pi.h"

#define ROBOT_WHEEL_RADIUS_M   0.016f   // 16 mm radius
#define ROBOT_WHEEL_BASE_M     0.0765f   // 76,5 mm track width

#define ROBOT_MAX_WHEEL_SPEED_RAD_S  125.0f
#define ROBOT_MAX_LINEAR_VEL_M_S     2.0f // max linear = r * 125 = 2
#define ROBOT_MAX_ANGULAR_VEL_RAD_S  52.29f // max angular = r * 2 * 125 / L = 52.29

typedef struct
{
    Motor       *motor_left;
    Motor       *motor_right;
    Encoder     *encoder_left;
    Encoder     *encoder_right;
    VelocityPI  *pi_left;
    VelocityPI  *pi_right;

    float cmd_linear_x_m_s;    // m/s, positive = forward
    float cmd_angular_z_rad_s; // rad/s, positive = CCW

    float pose_x_m;            // metres
    float pose_y_m;            // metres
    float pose_theta_rad;      // radians, wrapped to (-pi, pi]

    float setpoint_left_rad_s;
    float setpoint_right_rad_s;

    float velocity_left_rad_s;
    float velocity_right_rad_s;

} RobotController;

void RobotController_Init(RobotController *robot,
                          	Motor       *motor_left,
							Motor       *motor_right,
							Encoder     *encoder_left,
							Encoder     *encoder_right,
							VelocityPI  *pi_left,
							VelocityPI  *pi_right);

void RobotController_SetCmdVel(RobotController *robot,
                               float linear_x_m_s,
                               float angular_z_rad_s);

void RobotController_Update(RobotController *robot, float dt_s);

void RobotController_ResetOdometry(RobotController *robot);

void RobotController_Stop(RobotController *robot);


float RobotController_GetPoseX(const RobotController *robot);
float RobotController_GetPoseY(const RobotController *robot);
float RobotController_GetPoseTheta(const RobotController *robot);

float RobotController_GetLinearVelocity(const RobotController *robot);
float RobotController_GetAngularVelocity(const RobotController *robot);

#endif /* INC_ROBOT_CONTROLLER_H_ */
