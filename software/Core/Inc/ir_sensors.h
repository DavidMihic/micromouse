/*
 * ir_sensors.h
 *
 *  Created on: May 28, 2026
 *      Author: andrija
 */

#ifndef INC_IR_SENSORS_H_
#define INC_IR_SENSORS_H_


#include <stdint.h>
#include <stdbool.h>
#include "ir_array.h"

typedef struct
{
    IR_Gpio_t a0;
    IR_Gpio_t a1;
    IR_Gpio_t a2;
    IR_Gpio_t en;

    bool en_active_high;
} IR_DemuxPins_t;

IR_Status_t IR_Sensors_Init(ADC_HandleTypeDef *hadc1,
                            ADC_HandleTypeDef *hadc2,
                            TIM_HandleTypeDef *htim16,
                            const IR_DemuxPins_t *demux_pins);

IR_Status_t IR_Sensors_StartFrame(void);
void IR_Sensors_Stop(void);

bool IR_Sensors_IsBusy(void);
bool IR_Sensors_FrameReady(void);
void IR_Sensors_ClearFrameReady(void);

const int32_t *IR_Sensors_GetSignal(void);
const uint8_t *IR_Sensors_GetDigitalSignal(void);
const uint16_t *IR_Sensors_GetAmbient(void);
const uint16_t *IR_Sensors_GetLit(void);

IR_Status_t IR_Sensors_GetLastError(void);
uint32_t IR_Sensors_GetTimeoutCount(void);

void IR_Sensors_OnAdcConvCplt(ADC_HandleTypeDef *hadc);
void IR_Sensors_OnTimerElapsed(TIM_HandleTypeDef *htim);

IR_Array_t *IR_Sensors_GetHandle(void);

#endif /* INC_IR_SENSORS_H_ */
