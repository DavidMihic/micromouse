/*
 * button.h
 *
 *  Created on: May 28, 2026.
 *      Author: david
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stm32g4xx_hal.h"
#include <stdbool.h>

typedef enum
{
    BUTTON_OK = 0,
    BUTTON_ERROR
} Button_Status;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    volatile bool was_clicked;
} Button;

typedef struct
{
    GPIO_TypeDef *port_sw1; uint16_t pin_sw1;
    GPIO_TypeDef *port_sw2; uint16_t pin_sw2;
    GPIO_TypeDef *port_sw3; uint16_t pin_sw3;
    GPIO_PinState on_state;
} DipSwitch;

Button_Status Button_Init(Button *btn, GPIO_TypeDef *port, uint16_t pin);

Button_Status DipSwitch_Init(DipSwitch *dip,
                             GPIO_TypeDef *port1, uint16_t pin1,
                             GPIO_TypeDef *port2, uint16_t pin2,
                             GPIO_TypeDef *port3, uint16_t pin3,
                             GPIO_PinState on_state);

bool Button_HasClickedEvent(Button *btn);

bool DipSwitch_IsOn(const DipSwitch *dip, uint8_t switch_num);
uint8_t DipSwitch_GetValue(const DipSwitch *dip);

void Button_EXTI_Callback(Button *btn, uint16_t GPIO_Pin, TIM_HandleTypeDef *htim);
void Button_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_BUTTON_H_ */
