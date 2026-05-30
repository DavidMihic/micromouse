/*
 * neopixel.c
 *
 * Created on: May 27, 2026
 * Author: david
 */

#include "neopixel.h"
#include <string.h>

// Nowhere near max brightness to reduce current
const NeoPixel_Color COLOR_RED 		 = { .red = 100, .green = 0,   .blue = 0 };
const NeoPixel_Color COLOR_GREEN       = { .red = 0,   .green = 100, .blue = 0 };
const NeoPixel_Color COLOR_BLUE        = { .red = 0,   .green = 0,   .blue = 100 };
const NeoPixel_Color COLOR_YELLOW      = { .red = 100, .green = 100, .blue = 0 };
const NeoPixel_Color COLOR_ORANGE      = { .red = 80,  .green = 20,  .blue = 0 };
const NeoPixel_Color COLOR_MAGENTA     = { .red = 100, .green = 0,   .blue = 100};

static uint32_t _get_pwm_top(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

static uint16_t _get_pwm_hi_ticks(TIM_HandleTypeDef *htim)
{
    // Logic 1: ~64% duty cycle
    return (uint16_t)((_get_pwm_top(htim) * 64U) / 100U);
}

static uint16_t _get_pwm_lo_ticks(TIM_HandleTypeDef *htim)
{
    // Logic 0: ~32% duty cycle
    return (uint16_t)((_get_pwm_top(htim) * 32U) / 100U);
}

NeoPixel_Status NeoPixel_Init(NeoPixel *pixel, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (pixel == NULL || htim == NULL)
        return NEOPIXEL_ERROR;

    pixel->htim = htim;
    pixel->channel = channel;
    pixel->is_transferring = 0;

    memset(pixel->dma_buffer, 0, sizeof(pixel->dma_buffer));
    pixel->color.red = 0;
    pixel->color.green = 0;
    pixel->color.blue = 0;

    return NEOPIXEL_OK;
}

NeoPixel_Status NeoPixel_SetColorRGB(NeoPixel *pixel, uint8_t r, uint8_t g, uint8_t b)
{
    if (pixel == NULL)
        return NEOPIXEL_ERROR;

    pixel->color.red = r;
    pixel->color.green = g;
    pixel->color.blue = b;

    return NEOPIXEL_OK;
}

NeoPixel_Status NeoPixel_SetColor(NeoPixel *pixel, NeoPixel_Color color)
{
	if (pixel == NULL)
		return NEOPIXEL_ERROR;

	pixel->color = color;

	return NEOPIXEL_OK;
}

NeoPixel_Status NeoPixel_Off(NeoPixel *pixel)
{
	NeoPixel_SetColorRGB(pixel, 0, 0, 0);
    return NeoPixel_Show(pixel);
}

NeoPixel_Status NeoPixel_Show(NeoPixel *pixel)
{
    if (pixel == NULL || pixel->htim == NULL)
        return NEOPIXEL_ERROR;

    if (pixel->is_transferring)
        return NEOPIXEL_BUSY;

    uint16_t pwm_hi = _get_pwm_hi_ticks(pixel->htim);
    uint16_t pwm_lo = _get_pwm_lo_ticks(pixel->htim);

    uint32_t color_word = (pixel->color.green << 16) | (pixel->color.red << 8) | pixel->color.blue;

    for (int8_t bit = 23; bit >= 0; bit--)
    {
        if (color_word & (1UL << bit))
            pixel->dma_buffer[23 - bit] = pwm_hi;
        else
            pixel->dma_buffer[23 - bit] = pwm_lo;
    }

    pixel->is_transferring = 1;

    if (HAL_TIM_PWM_Start_DMA(pixel->htim, pixel->channel, (uint32_t *)pixel->dma_buffer, NEOPIXEL_BUFFER_SIZE) != HAL_OK)
    {
        pixel->is_transferring = 0;
        return NEOPIXEL_ERROR;
    }

    return NEOPIXEL_OK;
}

void NeoPixel_DMA_Callback(NeoPixel *pixel, TIM_HandleTypeDef *htim)
{
    if (pixel == NULL || htim == NULL)
        return;

    if (htim == pixel->htim)
    {
        HAL_TIM_PWM_Stop_DMA(pixel->htim, pixel->channel);
        __HAL_TIM_SET_COMPARE(pixel->htim, pixel->channel, 0);
        pixel->is_transferring = 0;
    }
}
