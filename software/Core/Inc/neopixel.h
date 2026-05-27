/*
 * neopixel.h
 *
 * Created on: May 27, 2026
 * Author: david
 */

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

// Definiramo fiksni broj LED-ica (budući da imate samo 1 statusnu LED-icu)
#define NEOPIXEL_COUNT 1

// WS2812 zahtijeva RESET signal u trajanju od minimalno 50 mikrosekundi (45 PWM slotova nule)
#define NEOPIXEL_RESET_SLOTS 45
#define NEOPIXEL_BUFFER_SIZE ((NEOPIXEL_COUNT * 24) + NEOPIXEL_RESET_SLOTS)

typedef enum
{
    NEOPIXEL_OK = 0,
    NEOPIXEL_ERROR,
    NEOPIXEL_BUSY
} NeoPixel_Status;

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} NeoPixel_Color;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    
    NeoPixel_Color color;
    uint16_t dma_buffer[NEOPIXEL_BUFFER_SIZE];
    volatile uint8_t is_transferring;
} NeoPixel;

extern const NeoPixel_Color COLOR_OFF;
extern const NeoPixel_Color COLOR_RED;
extern const NeoPixel_Color COLOR_GREEN;
extern const NeoPixel_Color COLOR_BLUE;
extern const NeoPixel_Color COLOR_YELLOW;
extern const NeoPixel_Color COLOR_ORANGE;
extern const NeoPixel_Color COLOR_MAGENTA;

NeoPixel_Status NeoPixel_Init(NeoPixel *pixel, TIM_HandleTypeDef *htim, uint32_t channel);

NeoPixel_Status NeoPixel_SetColorRGB(NeoPixel *pixel, uint8_t r, uint8_t g, uint8_t b);
NeoPixel_Status NeoPixel_SetColor(NeoPixel *pixel, NeoPixel_Color color);
NeoPixel_Status NeoPixel_Clear(NeoPixel *pixel);
NeoPixel_Status NeoPixel_Show(NeoPixel *pixel);

// Ova funkcija se poziva unutar HAL_TIM_PWM_PulseFinishedCallback prekidne rutine
void NeoPixel_DMA_Callback(NeoPixel *pixel, TIM_HandleTypeDef *htim);

#endif /* INC_NEOPIXEL_H_ */
