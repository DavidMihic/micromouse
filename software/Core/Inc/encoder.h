/*
 * encoder.h
 *
 *  Created on: May 27, 2026
 *      Author: andrija
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef enum
{
	ENCODER_OK = 0,
	ENCODER_ERROR
} Encoder_Status;

typedef enum
{
    ENCODER_COUNTER_16BIT = 0,
    ENCODER_COUNTER_32BIT
} Encoder_CounterBits;

typedef struct
{
	TIM_HandleTypeDef *htim;
	Encoder_CounterBits counter_bits;

	int8_t direction;

	uint32_t counts_per_rev;
	uint32_t last_raw_count;

    int32_t delta_count;
    int64_t position_count;

    float speed_cps;    // counts per second
    float speed_rps;    // revolutions per second
    float speed_rpm;    // revolutions per minute
    float speed_rad_s;  // rad/s
} Encoder;

Encoder_Status Encoder_Init(Encoder *enc,
                              TIM_HandleTypeDef *htim,
                              Encoder_CounterBits counter_bits,
                              uint32_t counts_per_rev,
                              int8_t direction);

void Encoder_Reset(Encoder *enc);
void Encoder_Update(Encoder *enc, float dt_s);

int64_t Encoder_GetPositionCounts(const Encoder *enc);
int32_t Encoder_GetDeltaCounts(const Encoder *enc);

float Encoder_GetSpeedCps(const Encoder *enc);
float Encoder_GetSpeedRps(const Encoder *enc);
float Encoder_GetSpeedRpm(const Encoder *enc);
float Encoder_GetSpeedRadS(const Encoder *enc);

uint32_t Encoder_GetRawCounter(const Encoder *enc);

#endif /* INC_ENCODER_H_ */
