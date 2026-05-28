/*
 * velocity_pi.c
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */
#include "velocity_pi.h"

static float clamp_float(float x, float min_val, float max_val)
{
    if (x > max_val)
        return max_val;

    if (x < min_val)
        return min_val;

    return x;
}

void VelocityPI_Init(VelocityPI *pi,
                     float kp,
                     float ki,
                     float dt_s,
                     float output_min,
                     float output_max)
{
    if (pi == NULL)
        return;

    pi->kp = kp;
    pi->ki = ki;
    pi->dt = dt_s;

    pi->integrator = 0.0f;

    pi->output_min = output_min;
    pi->output_max = output_max;

    pi->setpoint = 0.0f;
    pi->measurement = 0.0f;
    pi->error = 0.0f;
    pi->output = 0.0f;
}

void VelocityPI_Reset(VelocityPI *pi)
{
    if (pi == NULL)
        return;

    pi->integrator = 0.0f;
    pi->error = 0.0f;
    pi->output = 0.0f;
}

void VelocityPI_SetGains(VelocityPI *pi, float kp, float ki)
{
    if (pi == NULL)
        return;

    pi->kp = kp;
    pi->ki = ki;
}

void VelocityPI_SetSetpoint(VelocityPI *pi, float setpoint)
{
    if (pi == NULL)
        return;

    pi->setpoint = setpoint;
}

float VelocityPI_Update(VelocityPI *pi, float measurement)
{
    if (pi == NULL)
        return 0.0f;

    pi->measurement = measurement;
    pi->error = pi->setpoint - pi->measurement;

    float proportional = pi->kp * pi->error;

//  Update
    pi->integrator += pi->ki * pi->error * pi->dt;

//  Anti-windup
    pi->integrator = clamp_float(pi->integrator,
                                 pi->output_min,
                                 pi->output_max);

    float raw_output = proportional + pi->integrator;

    pi->output = clamp_float(raw_output,
                             pi->output_min,
                             pi->output_max);

    return pi->output;
}
