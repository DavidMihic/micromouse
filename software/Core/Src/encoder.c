#include "encoder.h"

#define ENCODER_TWO_PI 6.28318530718f

static uint32_t Encoder_ReadCounter(const Encoder *enc)
{
    return (uint32_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

static uint32_t Encoder_ReadAutoReload(const Encoder *enc)
{
    return (uint32_t)(enc->htim->Instance->ARR);
}

static int32_t Encoder_ComputeWrappedDelta(uint32_t now,
                                           uint32_t last,
                                           uint32_t counter_max)
{
    /*
     * Counter range is 0 ... counter_max.
     *
     * 16-bit timer: counter_max = 0xFFFF, range = 65536
     * 32-bit timer: counter_max = 0xFFFFFFFF, range = 4294967296
     *
     * Encoder_Update() must be called often enough that the true movement
     * between updates is less than half the timer range.
     */
    const uint64_t range = (uint64_t)counter_max + 1ULL;

    uint64_t forward_delta;

    if (now >= last)
    {
        forward_delta = (uint64_t)(now - last);
    }
    else
    {
        forward_delta = ((uint64_t)counter_max - (uint64_t)last)
                      + (uint64_t)now
                      + 1ULL;
    }

    int64_t signed_delta;

    if (forward_delta >= (range / 2ULL))
    {
        signed_delta = (int64_t)forward_delta - (int64_t)range;
    }
    else
    {
        signed_delta = (int64_t)forward_delta;
    }

    return (int32_t)signed_delta;
}

Encoder_Status Encoder_Init(Encoder *enc,
                            TIM_HandleTypeDef *htim,
                            float ticks_per_rev,
                            int8_t direction,
                            float velocity_filter_tau_s)
{
    if (enc == NULL || htim == NULL || ticks_per_rev <= 0.0f)
    {
        return ENCODER_ERROR;
    }

    enc->htim = htim;
    enc->ticks_per_rev = ticks_per_rev;
    enc->direction = (direction < 0) ? -1 : 1;

    enc->counter_max = Encoder_ReadAutoReload(enc);

    if (enc->counter_max < 1U)
    {
        return ENCODER_ERROR;
    }

    enc->last_counter = 0U;
    enc->delta_ticks = 0;
    enc->position_ticks = 0;
    enc->position_rad = 0.0f;

    enc->raw_velocity_ticks_s = 0.0f;
    enc->raw_velocity_rad_s = 0.0f;
    enc->velocity_ticks_s = 0.0f;
    enc->velocity_rad_s = 0.0f;

    enc->velocity_filter_tau_s = (velocity_filter_tau_s > 0.0f) ? velocity_filter_tau_s : 0.0f;
    enc->velocity_filter_initialized = 0U;

    if (HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return ENCODER_ERROR;
    }

    Encoder_Reset(enc);

    return ENCODER_OK;
}

void Encoder_Reset(Encoder *enc)
{
    if (enc == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(enc->htim, 0U);

    enc->counter_max = Encoder_ReadAutoReload(enc);
    enc->last_counter = Encoder_ReadCounter(enc);

    enc->delta_ticks = 0;
    enc->position_ticks = 0;
    enc->position_rad = 0.0f;

    enc->raw_velocity_ticks_s = 0.0f;
    enc->raw_velocity_rad_s = 0.0f;
    enc->velocity_ticks_s = 0.0f;
    enc->velocity_rad_s = 0.0f;
    enc->velocity_filter_initialized = 0U;
}

void Encoder_Update(Encoder *enc, float dt_s)
{
    if (enc == NULL)
    {
        return;
    }

    const uint32_t now = Encoder_ReadCounter(enc);

    int32_t delta = Encoder_ComputeWrappedDelta(now,
                                                enc->last_counter,
                                                enc->counter_max);

    delta *= enc->direction;

    enc->last_counter = now;
    enc->delta_ticks = delta;
    enc->position_ticks += (int64_t)delta;

    enc->position_rad =
        ((float)enc->position_ticks / enc->ticks_per_rev) * ENCODER_TWO_PI;

    if (dt_s <= 0.0f)
    {
        enc->raw_velocity_ticks_s = 0.0f;
        enc->raw_velocity_rad_s = 0.0f;
        enc->velocity_ticks_s = 0.0f;
        enc->velocity_rad_s = 0.0f;
        enc->velocity_filter_initialized = 0U;
        return;
    }

    /* Raw velocity from the latest encoder increment. */
    enc->raw_velocity_ticks_s = (float)delta / dt_s;
    enc->raw_velocity_rad_s =
        (enc->raw_velocity_ticks_s / enc->ticks_per_rev) * ENCODER_TWO_PI;

    /* Filter disabled. */
    if (enc->velocity_filter_tau_s <= 0.0f)
    {
        enc->velocity_ticks_s = enc->raw_velocity_ticks_s;
        enc->velocity_rad_s = enc->raw_velocity_rad_s;
        enc->velocity_filter_initialized = 1U;
        return;
    }

    /* Initialize filter at the first real measurement to avoid a fake ramp from zero. */
    if (enc->velocity_filter_initialized == 0U)
    {
        enc->velocity_ticks_s = enc->raw_velocity_ticks_s;
        enc->velocity_rad_s = enc->raw_velocity_rad_s;
        enc->velocity_filter_initialized = 1U;
        return;
    }

    const float alpha = dt_s / (enc->velocity_filter_tau_s + dt_s);

    enc->velocity_ticks_s += alpha *
        (enc->raw_velocity_ticks_s - enc->velocity_ticks_s);

    enc->velocity_rad_s += alpha *
        (enc->raw_velocity_rad_s - enc->velocity_rad_s);
}

void Encoder_SetVelocityFilterTau(Encoder *enc, float tau_s)
{
    if (enc == NULL)
    {
        return;
    }

    enc->velocity_filter_tau_s = (tau_s > 0.0f) ? tau_s : 0.0f;
    enc->velocity_filter_initialized = 0U;
}

void Encoder_ResetVelocityFilter(Encoder *enc)
{
    if (enc == NULL)
    {
        return;
    }

    enc->velocity_ticks_s = enc->raw_velocity_ticks_s;
    enc->velocity_rad_s = enc->raw_velocity_rad_s;
    enc->velocity_filter_initialized = 1U;
}

int64_t Encoder_GetPositionTicks(const Encoder *enc)
{
    return (enc != NULL) ? enc->position_ticks : 0;
}

int32_t Encoder_GetDeltaTicks(const Encoder *enc)
{
    return (enc != NULL) ? enc->delta_ticks : 0;
}

float Encoder_GetPositionRad(const Encoder *enc)
{
    return (enc != NULL) ? enc->position_rad : 0.0f;
}

float Encoder_GetRawVelocityTicksPerSecond(const Encoder *enc)
{
    return (enc != NULL) ? enc->raw_velocity_ticks_s : 0.0f;
}

float Encoder_GetRawVelocityRadPerSecond(const Encoder *enc)
{
    return (enc != NULL) ? enc->raw_velocity_rad_s : 0.0f;
}

float Encoder_GetVelocityTicksPerSecond(const Encoder *enc)
{
    return (enc != NULL) ? enc->velocity_ticks_s : 0.0f;
}

float Encoder_GetVelocityRadPerSecond(const Encoder *enc)
{
    return (enc != NULL) ? enc->velocity_rad_s : 0.0f;
}

uint32_t Encoder_GetRawCounter(const Encoder *enc)
{
    return (enc != NULL) ? Encoder_ReadCounter(enc) : 0U;
}
