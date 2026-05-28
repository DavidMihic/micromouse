/*
 * button.c
 *
 *  Created on: May 28, 2026.
 *      Author: david
 */

#include "button.h"

static Button *_button_pending = NULL;

Button_Status Button_Init(Button *btn, GPIO_TypeDef *port, uint16_t pin)
{
    if (btn == NULL || port == NULL)
        return BUTTON_ERROR;

    btn->port = port;
    btn->pin = pin;
    btn->was_clicked = false;

    return BUTTON_OK;
}

bool Button_HasClickedEvent(Button *btn)
{
    if (btn == NULL)
        return false;

    if (btn->was_clicked)
    {
        btn->was_clicked = false;
        return true;
    }
    return false;
}

void Button_EXTI_Callback(Button *btn, uint16_t GPIO_Pin, TIM_HandleTypeDef *htim)
{
    if (btn == NULL || htim == NULL || btn->port == NULL)
        return;

    if (GPIO_Pin != btn->pin)
    	return;

    if (_button_pending != NULL)
    	return;

    _button_pending = btn;

	if (btn->pin == GPIO_PIN_4)			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	else if (btn->pin == GPIO_PIN_13)	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
	htim->State = HAL_TIM_STATE_READY;
	HAL_TIM_Base_Start_IT(htim);
}

void Button_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == NULL)
		return;

    HAL_TIM_Base_Stop_IT(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    htim->State = HAL_TIM_STATE_READY;

    Button *pending = _button_pending;
    _button_pending = NULL;

    if (pending == NULL || pending->port == NULL)
    	return;

    GPIO_PinState current_state = HAL_GPIO_ReadPin(pending->port, pending->pin);

	if (current_state == GPIO_PIN_RESET)
        pending->was_clicked = true;

    __HAL_GPIO_EXTI_CLEAR_IT(pending->pin);

    if (pending->pin == GPIO_PIN_4)			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    else if (pending->pin == GPIO_PIN_13)	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

Button_Status DipSwitch_Init(DipSwitch *dip,
                             GPIO_TypeDef *port1, uint16_t pin1,
                             GPIO_TypeDef *port2, uint16_t pin2,
                             GPIO_TypeDef *port3, uint16_t pin3,
                             GPIO_PinState on_state)
{
    if (dip == NULL || port1 == NULL || port2 == NULL || port3 == NULL)
        return BUTTON_ERROR;

    dip->port_sw1 = port1; dip->pin_sw1 = pin1;
    dip->port_sw2 = port2; dip->pin_sw2 = pin2;
    dip->port_sw3 = port3; dip->pin_sw3 = pin3;
    dip->on_state = on_state;

    return BUTTON_OK;
}

bool DipSwitch_IsOn(const DipSwitch *dip, uint8_t switch_num)
{
    if (dip == NULL)
        return false;

    GPIO_PinState state = GPIO_PIN_RESET;

    if (switch_num == 1)		state = HAL_GPIO_ReadPin(dip->port_sw1, dip->pin_sw1);
    else if (switch_num == 2)	state = HAL_GPIO_ReadPin(dip->port_sw2, dip->pin_sw2);
    else if (switch_num == 3)	state = HAL_GPIO_ReadPin(dip->port_sw3, dip->pin_sw3);
    else return false;

    return (state == dip->on_state);
}

uint8_t DipSwitch_GetValue(const DipSwitch *dip)
{
    if (dip == NULL)
        return 0;

    uint8_t value = 0;

    if (DipSwitch_IsOn(dip, 1)) value |= (1U << 0);
    if (DipSwitch_IsOn(dip, 2)) value |= (1U << 1);
    if (DipSwitch_IsOn(dip, 3)) value |= (1U << 2);

    return value;
}
