/*
 * ir_sensors.c
 *
 *  Created on: May 28, 2026
 *      Author: andrija
 */


#include "ir_sensors.h"
#include <stddef.h>

/*
 * Adjust these if your CubeMX ADC rank order is different.
 *
 * This assumes:
 *   ADC1 rank 1..4 = IR_REC_1..IR_REC_4
 *   ADC2 rank 1..2 = IR_REC_5..IR_REC_6
 */
#define IR_ADC1_CHANNEL_COUNT 4u
#define IR_ADC2_CHANNEL_COUNT 2u

/*
 * Timing values.
 *
 * Keep max_pulse_us short so the IR LEDs cannot remain on for long.
 */
#define IR_PULSE_SETTLE_US   100u
#define IR_MAX_PULSE_US      200u
#define IR_ADC_TIMEOUT_US    1000u

static IR_Array_t s_ir;

static uint16_t s_adc1_dma_buf[IR_ADC1_CHANNEL_COUNT];
static uint16_t s_adc2_dma_buf[IR_ADC2_CHANNEL_COUNT];

static bool s_initialized = false;

IR_Status_t IR_Sensors_Init(ADC_HandleTypeDef *hadc1,
                            ADC_HandleTypeDef *hadc2,
                            TIM_HandleTypeDef *htim16,
                            const IR_DemuxPins_t *demux_pins)
{
    IR_Array_Init_t init = {0};
    IR_Status_t status;

    if ((hadc1 == NULL) || (hadc2 == NULL) || (htim16 == NULL) || (demux_pins == NULL))
    {
        return IR_BAD_PARAM;
    }

    /*
     * Optional, but useful. Remove these two calls if you already calibrate ADCs elsewhere.
     */
    if (HAL_ADCEx_Calibration_Start(hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return IR_HAL_ERROR;
    }

    if (HAL_ADCEx_Calibration_Start(hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return IR_HAL_ERROR;
    }

    init.hadc1 = hadc1;
    init.hadc2 = hadc2;
    init.htim_us = htim16;

    init.demux_a0 = demux_pins->a0;
    init.demux_a1 = demux_pins->a1;
    init.demux_a2 = demux_pins->a2;
    init.demux_en = demux_pins->en;
    init.demux_en_active_high = demux_pins->en_active_high;

    init.adc1_buf = s_adc1_dma_buf;
    init.adc1_len = IR_ADC1_CHANNEL_COUNT;

    init.adc2_buf = s_adc2_dma_buf;
    init.adc2_len = IR_ADC2_CHANNEL_COUNT;

    /*
     * LED 1 -> IR_REC_1
     * LED 2 -> IR_REC_2
     * ...
     * LED 6 -> IR_REC_6
     *
     * rank_index is zero-based:
     *   ADC rank 1 -> index 0
     *   ADC rank 2 -> index 1
     */
    init.sensor_map[0] = (IR_ChannelMap_t){IR_ADC_2, 0};
    init.sensor_map[1] = (IR_ChannelMap_t){IR_ADC_1, 0};
    init.sensor_map[2] = (IR_ChannelMap_t){IR_ADC_1, 1};
    init.sensor_map[3] = (IR_ChannelMap_t){IR_ADC_1, 2};
    init.sensor_map[4] = (IR_ChannelMap_t){IR_ADC_1, 3};
    init.sensor_map[5] = (IR_ChannelMap_t){IR_ADC_2, 1};

    init.pulse_settle_us = IR_PULSE_SETTLE_US;
    init.max_pulse_us = IR_MAX_PULSE_US;
    init.adc_timeout_us = IR_ADC_TIMEOUT_US;

    init.frame_done_cb = NULL;

    status = IR_Array_Init(&s_ir, &init);

    if (status == IR_OK)
    {
        s_initialized = true;
    }
    else
    {
        s_initialized = false;
    }

    return status;
}

IR_Status_t IR_Sensors_StartFrame(void)
{
    if (!s_initialized)
    {
        return IR_BAD_PARAM;
    }

    return IR_Array_StartFrame(&s_ir);
}

void IR_Sensors_Stop(void)
{
    if (!s_initialized)
    {
        return;
    }

    IR_Array_Stop(&s_ir);
}

bool IR_Sensors_IsBusy(void)
{
    if (!s_initialized)
    {
        return false;
    }

    return IR_Array_IsBusy(&s_ir);
}

bool IR_Sensors_FrameReady(void)
{
    if (!s_initialized)
    {
        return false;
    }

    return IR_Array_FrameReady(&s_ir);
}

void IR_Sensors_ClearFrameReady(void)
{
    if (!s_initialized)
    {
        return;
    }

    IR_Array_ClearFrameReady(&s_ir);
}

const int32_t *IR_Sensors_GetSignal(void)
{
    return s_ir.signal;
}

const uint8_t *IR_Sensors_GetDigitalSignal(void)
{
	return s_ir.digital_signal;
}


IR_Status_t IR_Sensors_GetLastError(void)
{
    return s_ir.last_error;
}

uint32_t IR_Sensors_GetTimeoutCount(void)
{
    return s_ir.timeout_count;
}

void IR_Sensors_OnAdcConvCplt(ADC_HandleTypeDef *hadc)
{
    if (!s_initialized)
    {
        return;
    }

    IR_Array_OnAdcConvCplt(&s_ir, hadc);
}

void IR_Sensors_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (!s_initialized)
    {
        return;
    }

    IR_Array_OnTimerElapsed(&s_ir, htim);
}

IR_Array_t *IR_Sensors_GetHandle(void)
{
    return &s_ir;
}
