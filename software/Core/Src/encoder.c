/*
 * encoder.c
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */

#include "encoder.h"

#define ENCODER_TWO_PI 6.28318530718f

static uint32_t encoder_read_raw(const Encoder *enc)
{
    return __HAL_TIM_GET_COUNTER(enc->htim);
}

Encoder_Status Encoder_Init(Encoder *enc,
                              TIM_HandleTypeDef *htim,
                              Encoder_CounterBits counter_bits,
                              uint32_t counts_per_rev,
                              int8_t direction)
{
    if (enc == NULL || htim == NULL)
        return ENCODER_ERROR;

    enc->htim = htim;
    enc->counter_bits = counter_bits;
    enc->counts_per_rev = counts_per_rev;

    if (direction == -1)
        enc->direction = -1;
    else
        enc->direction = 1;

    enc->last_raw_count = 0;
    enc->delta_count = 0;
    enc->position_count = 0;

    enc->speed_cps = 0.0f;
    enc->speed_rps = 0.0f;
    enc->speed_rpm = 0.0f;
    enc->speed_rad_s = 0.0f;

    if (HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL) != HAL_OK)
        return ENCODER_ERROR;

    Encoder_Reset(enc);

    return ENCODER_OK;
}

void Encoder_Reset(Encoder *enc)
{
    if (enc == NULL)
        return;

    __HAL_TIM_SET_COUNTER(enc->htim, 0);

    enc->last_raw_count = encoder_read_raw(enc);
    enc->delta_count = 0;
    enc->position_count = 0;

    enc->speed_cps = 0.0f;
    enc->speed_rps = 0.0f;
    enc->speed_rpm = 0.0f;
    enc->speed_rad_s = 0.0f;
}

void Encoder_Update(Encoder *enc, float dt_s)
{
    if (enc == NULL)
        return;

    uint32_t raw_count = encoder_read_raw(enc);

    int32_t delta;

    if (enc->counter_bits == ENCODER_COUNTER_32BIT)
    {
        /*
         * Works correctly with 32-bit timer period = 0xFFFFFFFF.
         */
        delta = (int32_t)(raw_count - enc->last_raw_count);
    }
    else
    {
        /*
         * Works correctly with 16-bit timer period = 0xFFFF.
         * This handles counter overflow/underflow automatically.
         */
        delta = (int16_t)((uint16_t)raw_count - (uint16_t)enc->last_raw_count);
    }

    delta *= enc->direction;

    enc->last_raw_count = raw_count;
    enc->delta_count = delta;
    enc->position_count += delta;

    if (dt_s > 0.0f)
    {
        enc->speed_cps = (float)delta / dt_s;

        if (enc->counts_per_rev > 0U)
        {
            enc->speed_rps = enc->speed_cps / (float)enc->counts_per_rev;
            enc->speed_rpm = enc->speed_rps * 60.0f;
            enc->speed_rad_s = enc->speed_rps * ENCODER_TWO_PI;
        }
        else
        {
            enc->speed_rps = 0.0f;
            enc->speed_rpm = 0.0f;
            enc->speed_rad_s = 0.0f;
        }
    }
}

int64_t Encoder_GetPositionCounts(const Encoder *enc)
{
    if (enc == NULL)
        return 0;

    return enc->position_count;
}

int32_t Encoder_GetDeltaCounts(const Encoder *enc)
{
    if (enc == NULL)
        return 0;

    return enc->delta_count;
}

float Encoder_GetSpeedCps(const Encoder *enc)
{
    if (enc == NULL)
        return 0.0f;

    return enc->speed_cps;
}

float Encoder_GetSpeedRps(const Encoder *enc)
{
    if (enc == NULL)
        return 0.0f;

    return enc->speed_rps;
}

float Encoder_GetSpeedRpm(const Encoder *enc)
{
    if (enc == NULL)
        return 0.0f;

    return enc->speed_rpm;
}

float Encoder_GetSpeedRadS(const Encoder *enc)
{
    if (enc == NULL)
        return 0.0f;

    return enc->speed_rad_s;
}

uint32_t Encoder_GetRawCounter(const Encoder *enc)
{
    if (enc == NULL)
        return 0;

    return encoder_read_raw(enc);
}
