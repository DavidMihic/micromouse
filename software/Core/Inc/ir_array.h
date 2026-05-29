/*
 * ir_array.h
 *
 *  Created on: May 28, 2026
 *      Author: andrija
 */

#ifndef INC_IR_ARRAY_H_
#define INC_IR_ARRAY_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

#define IR_ARRAY_SENSOR_COUNT 6u
#define IR_SEN_DIGTAL_TRESHOLD 1500

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} IR_Gpio_t;

typedef enum
{
    IR_ADC_NONE = 0,
    IR_ADC_1    = 1,
    IR_ADC_2    = 2
} IR_AdcId_t;

typedef struct
{
    IR_AdcId_t adc;
    uint8_t rank_index;   // zero-based DMA buffer index
} IR_ChannelMap_t;

typedef enum
{
    IR_OK = 0,
    IR_BUSY,
    IR_BAD_PARAM,
    IR_HAL_ERROR,
    IR_TIMEOUT
} IR_Status_t;

typedef enum
{
    IR_STATE_IDLE = 0,
    IR_STATE_PULSE_SETTLE,
    IR_STATE_LIT_ADC,
    IR_STATE_DONE,
    IR_STATE_FAULT
} IR_State_t;

typedef struct IR_Array IR_Array_t;

typedef void (*IR_FrameDoneCallback_t)(IR_Array_t *ir);

typedef struct
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
    TIM_HandleTypeDef *htim_us;

    IR_Gpio_t demux_a0;
    IR_Gpio_t demux_a1;
    IR_Gpio_t demux_a2;
    IR_Gpio_t demux_en;

    bool demux_en_active_high;

    uint16_t *adc1_buf;
    uint8_t adc1_len;

    uint16_t *adc2_buf;
    uint8_t adc2_len;

    IR_ChannelMap_t sensor_map[IR_ARRAY_SENSOR_COUNT];

    uint16_t pulse_settle_us;
    uint16_t max_pulse_us;
    uint16_t adc_timeout_us;

    IR_FrameDoneCallback_t frame_done_cb;
} IR_Array_Init_t;

struct IR_Array
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
    TIM_HandleTypeDef *htim_us;

    IR_Gpio_t demux_a0;
    IR_Gpio_t demux_a1;
    IR_Gpio_t demux_a2;
    IR_Gpio_t demux_en;

    bool demux_en_active_high;

    uint16_t *adc1_buf;
    uint8_t adc1_len;

    uint16_t *adc2_buf;
    uint8_t adc2_len;

    IR_ChannelMap_t sensor_map[IR_ARRAY_SENSOR_COUNT];

    int32_t signal[IR_ARRAY_SENSOR_COUNT];
    uint8_t digital_signal[IR_ARRAY_SENSOR_COUNT];

    uint16_t pulse_settle_us;
    uint16_t max_pulse_us;
    uint16_t adc_timeout_us;

    volatile IR_State_t state;
    volatile IR_Status_t last_error;
    volatile bool busy;
    volatile bool frame_ready;
    volatile uint8_t current_sensor;

    volatile uint8_t adc_needed_mask;
    volatile uint8_t adc_done_mask;

    uint32_t timeout_count;

    IR_FrameDoneCallback_t frame_done_cb;
};

IR_Status_t IR_Array_Init(IR_Array_t *ir, const IR_Array_Init_t *init);
IR_Status_t IR_Array_StartFrame(IR_Array_t *ir);
void IR_Array_Stop(IR_Array_t *ir);

void IR_Array_AllEmittersOff(IR_Array_t *ir);

void IR_Array_OnAdcConvCplt(IR_Array_t *ir, ADC_HandleTypeDef *hadc);
void IR_Array_OnTimerElapsed(IR_Array_t *ir, TIM_HandleTypeDef *htim);

bool IR_Array_IsBusy(const IR_Array_t *ir);
bool IR_Array_FrameReady(const IR_Array_t *ir);
void IR_Array_ClearFrameReady(IR_Array_t *ir);



#endif /* INC_IR_ARRAY_H_ */
