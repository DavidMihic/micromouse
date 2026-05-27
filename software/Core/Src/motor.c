/*
 * motor.c
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */

#include "motor.h"

static int16_t clamp_command(int16_t command)
{
    if (command > MOTOR_PWM_MAX)
        return MOTOR_PWM_MAX;

    if (command < -MOTOR_PWM_MAX)
        return -MOTOR_PWM_MAX;

    return command;
}

static uint32_t get_pwm_top(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

static void set_pwm_scaled(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t value)
{
    if (value > MOTOR_PWM_MAX)
        value = MOTOR_PWM_MAX;

    uint32_t top = get_pwm_top(htim);

    uint32_t ccr = ((uint32_t)value * top) / MOTOR_PWM_MAX;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

static void set_pwm_percent_100(TIM_HandleTypeDef *htim, uint32_t channel)
{
    __HAL_TIM_SET_COMPARE(htim, channel, get_pwm_top(htim));
}

static void set_pwm_percent_0(TIM_HandleTypeDef *htim, uint32_t channel)
{
    __HAL_TIM_SET_COMPARE(htim, channel, 0);
}

Motor_Status Motor_Init(Motor *motor)
{
    if (motor == NULL)
        return MOTOR_ERROR;

    if (motor->htim_in1 == NULL || motor->htim_in2 == NULL)
        return MOTOR_ERROR;

    if (motor->direction != 1 && motor->direction != -1)
        motor->direction = 1;

    if (HAL_TIM_PWM_Start(motor->htim_in1, motor->channel_in1) != HAL_OK)
        return MOTOR_ERROR;

    if (HAL_TIM_PWM_Start(motor->htim_in2, motor->channel_in2) != HAL_OK)
        return MOTOR_ERROR;

    Motor_Brake(motor);

    return MOTOR_OK;
}

void Motor_Set(Motor *motor, int16_t command)
{
    if (motor == NULL)
        return;

    command = clamp_command(command);

    command *= motor->direction;

    if (command > 0)
    {
        /*
         * Forward drive/brake PWM:
         *
         * Drive: IN1 = 1, IN2 = 0
         * Brake: IN1 = 1, IN2 = 1
         *
         * Therefore:
         * IN1 = 100%
         * IN2 = inverted duty
         */
        uint16_t duty = (uint16_t)command;

        set_pwm_percent_100(motor->htim_in1, motor->channel_in1);
        set_pwm_scaled(motor->htim_in2, motor->channel_in2,
        			MOTOR_PWM_MAX - duty);
    }
    else if (command < 0)
    {
        /*
         * Reverse drive/brake PWM:
         *
         * Drive: IN1 = 0, IN2 = 1
         * Brake: IN1 = 1, IN2 = 1
         *
         * Therefore:
         * IN1 = inverted duty
         * IN2 = 100%
         */
        uint16_t duty = (uint16_t)(-command);

        set_pwm_scaled(motor->htim_in1, motor->channel_in1,
        			MOTOR_PWM_MAX - duty);
        set_pwm_percent_100(motor->htim_in2, motor->channel_in2);
    }
    else
    {
        /*
         * Zero command = active brake.
         */
        Motor_Brake(motor);
    }
}

void Motor_Brake(Motor *motor)
{
    if (motor == NULL)
        return;

    /*
     * IN1 = 1, IN2 = 1 -> brake
     */
    set_pwm_percent_100(motor->htim_in1, motor->channel_in1);
    set_pwm_percent_100(motor->htim_in2, motor->channel_in2);
}

void Motor_Coast(Motor *motor)
{
    if (motor == NULL)
        return;

    /*
     * IN1 = 0, IN2 = 0 -> coast
     * Not used during normal PWM control, only useful for disable/fault state.
     */
    set_pwm_percent_0(motor->htim_in1, motor->channel_in1);
    set_pwm_percent_0(motor->htim_in2, motor->channel_in2);
}

