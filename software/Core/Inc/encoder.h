#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

typedef enum
{
    ENCODER_OK = 0,
    ENCODER_ERROR
} Encoder_Status;

typedef struct
{
    TIM_HandleTypeDef *htim;

    /* Number of timer ticks for one wheel revolution. */
    float ticks_per_rev;

    /* +1 for normal direction, -1 to invert this encoder. */
    int8_t direction;

    /* Hardware counter information. CNT is read as uint32_t even for 16-bit timers. */
    uint32_t counter_max;      // usually 0xFFFF or 0xFFFFFFFF
    uint32_t last_counter;

    /* Position and per-update increment. */
    int32_t delta_ticks;
    int64_t position_ticks;
    float position_rad;

    /* Raw velocity directly from delta_ticks / dt. Useful for debugging. */
    float raw_velocity_ticks_s;
    float raw_velocity_rad_s;

    /* Filtered velocity. Encoder_GetVelocity...() returns these values. */
    float velocity_ticks_s;
    float velocity_rad_s;

    /* First-order velocity low-pass filter time constant in seconds. 0 disables filtering. */
    float velocity_filter_tau_s;
    uint8_t velocity_filter_initialized;

} Encoder;

Encoder_Status Encoder_Init(Encoder *enc,
                            TIM_HandleTypeDef *htim,
                            float ticks_per_rev,
                            int8_t direction,
                            float velocity_filter_tau_s);

void Encoder_Reset(Encoder *enc);
void Encoder_Update(Encoder *enc, float dt_s);

void Encoder_SetVelocityFilterTau(Encoder *enc, float tau_s);
void Encoder_ResetVelocityFilter(Encoder *enc);

int64_t  Encoder_GetPositionTicks(const Encoder *enc);
int32_t  Encoder_GetDeltaTicks(const Encoder *enc);
float    Encoder_GetPositionRad(const Encoder *enc);

float    Encoder_GetRawVelocityTicksPerSecond(const Encoder *enc);
float    Encoder_GetRawVelocityRadPerSecond(const Encoder *enc);
float    Encoder_GetVelocityTicksPerSecond(const Encoder *enc);
float    Encoder_GetVelocityRadPerSecond(const Encoder *enc);

uint32_t Encoder_GetRawCounter(const Encoder *enc);

#endif
