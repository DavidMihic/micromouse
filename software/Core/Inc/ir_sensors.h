/*
 * ir_sensors.h
 *
 *  Created on: May 28, 2026
 *      Author: andrija
 */

#ifndef INC_IR_SENSORS_H_
#define INC_IR_SENSORS_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define IR_NUM_SENSORS 6

typedef struct
{
    uint16_t ambient[IR_NUM_SENSORS];
    uint16_t active[IR_NUM_SENSORS];
    int16_t  signal[IR_NUM_SENSORS];

    bool scan_ready;
    uint32_t scan_count;
} IR_Data_t;

void IR_Init(ADC_HandleTypeDef *hadc1_, ADC_HandleTypeDef *hadc2_);

bool IR_StartScan(void);
void IR_Task(void);

bool IR_IsBusy(void);
bool IR_IsScanReady(void);
void IR_ClearScanReadyFlag(void);

const IR_Data_t* IR_GetData(void);

void IR_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#endif /* INC_IR_SENSORS_H_ */
