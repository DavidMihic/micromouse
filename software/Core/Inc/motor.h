/*
 * motors.h
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"

#define MOTOR_PWM_MAX 1000

typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERROR
} Motor_Status;

typedef struct
{
    TIM_HandleTypeDef *htim_in1;
    uint32_t channel_in1;

    TIM_HandleTypeDef *htim_in2;
    uint32_t channel_in2;

    int8_t direction;
} Motor;


Motor_Status Motor_Init(Motor *motor);

void Motor_Set(Motor *motor, int16_t command);
void Motor_Brake(Motor *motor);
void Motor_Coast(Motor *motor);

#endif /* INC_MOTOR_H_ */
