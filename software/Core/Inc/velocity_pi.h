/*
 * velocity_pi.h
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */

#ifndef INC_VELOCITY_PI_H_
#define INC_VELOCITY_PI_H_

#include "stm32g4xx_hal.h"

typedef struct
{
	float kp;
	float ki;

	float dt;

	float integrator;

	float output_min;
	float output_max;

	float setpoint;
	float measurement;
	float error;
	float output;
} VelocityPI;

void VelocityPI_Init(VelocityPI *pi,
                     float kp,
                     float ki,
                     float dt_s,
                     float output_min,
                     float output_max);

void VelocityPI_Reset(VelocityPI *pi);

void VelocityPI_SetGains(VelocityPI *pi, float kp, float ki);

void VelocityPI_SetSetpoint(VelocityPI *pi, float setpoint);

float VelocityPI_Update(VelocityPI *pi, float measurement);


#endif /* INC_VELOCITY_PI_H_ */
