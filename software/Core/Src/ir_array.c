#include "ir_array.h"
#include <string.h>

#define IR_ADC1_MASK 0x01u
#define IR_ADC2_MASK 0x02u

static void ir_write_pin(IR_Gpio_t p, bool high)
{
    HAL_GPIO_WritePin(p.port, p.pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ir_set_enable(IR_Array_t *ir, bool enable)
{
    bool level = ir->demux_en_active_high ? enable : !enable;
    ir_write_pin(ir->demux_en, level);
}

static void ir_set_address(IR_Array_t *ir, uint8_t index)
{
    ir_write_pin(ir->demux_a0, (index & 0x01u) != 0u);
    ir_write_pin(ir->demux_a1, (index & 0x02u) != 0u);
    ir_write_pin(ir->demux_a2, (index & 0x04u) != 0u);
}

void IR_Array_AllEmittersOff(IR_Array_t *ir)
{
    if (ir == NULL)
    {
        return;
    }

    ir_set_enable(ir, false);
    ir_set_address(ir, 0u);
}

static void ir_select_and_enable_led(IR_Array_t *ir, uint8_t index)
{
    /*
     * Disable first so the demux output cannot glitch while changing address.
     */
    ir_set_enable(ir, false);
    ir_set_address(ir, index);
    ir_set_enable(ir, true);
}

static IR_Status_t ir_timer_start_us(IR_Array_t *ir, uint16_t us)
{
    if ((ir == NULL) || (ir->htim_us == NULL) || (us == 0u))
    {
        return IR_BAD_PARAM;
    }

    HAL_TIM_Base_Stop_IT(ir->htim_us);

    __HAL_TIM_SET_COUNTER(ir->htim_us, 0u);
    __HAL_TIM_SET_AUTORELOAD(ir->htim_us, (uint32_t)us - 1u);
    __HAL_TIM_CLEAR_FLAG(ir->htim_us, TIM_FLAG_UPDATE);

    if (HAL_TIM_Base_Start_IT(ir->htim_us) != HAL_OK)
    {
        return IR_HAL_ERROR;
    }

    return IR_OK;
}

static void ir_timer_stop(IR_Array_t *ir)
{
    if ((ir != NULL) && (ir->htim_us != NULL))
    {
        HAL_TIM_Base_Stop_IT(ir->htim_us);
    }
}

static void ir_stop_adc(IR_Array_t *ir)
{
    if (ir == NULL)
    {
        return;
    }

    if (ir->hadc1 != NULL)
    {
        HAL_ADC_Stop_DMA(ir->hadc1);
    }

    if (ir->hadc2 != NULL)
    {
        HAL_ADC_Stop_DMA(ir->hadc2);
    }

    ir->adc_needed_mask = 0u;
    ir->adc_done_mask = 0u;
}

static IR_Status_t ir_start_adc(IR_Array_t *ir)
{
    if (ir == NULL)
    {
        return IR_BAD_PARAM;
    }

    ir->adc_needed_mask = 0u;
    ir->adc_done_mask = 0u;

    if ((ir->hadc1 != NULL) && (ir->adc1_buf != NULL) && (ir->adc1_len > 0u))
    {
        HAL_ADC_Stop_DMA(ir->hadc1);

        ir->adc_needed_mask |= IR_ADC1_MASK;

        if (HAL_ADC_Start_DMA(ir->hadc1,
                              (uint32_t *)ir->adc1_buf,
                              ir->adc1_len) != HAL_OK)
        {
            ir_stop_adc(ir);
            return IR_HAL_ERROR;
        }
    }

    if ((ir->hadc2 != NULL) && (ir->adc2_buf != NULL) && (ir->adc2_len > 0u))
    {
        HAL_ADC_Stop_DMA(ir->hadc2);

        ir->adc_needed_mask |= IR_ADC2_MASK;

        if (HAL_ADC_Start_DMA(ir->hadc2,
                              (uint32_t *)ir->adc2_buf,
                              ir->adc2_len) != HAL_OK)
        {
            ir_stop_adc(ir);
            return IR_HAL_ERROR;
        }
    }

    if (ir->adc_needed_mask == 0u)
    {
        return IR_BAD_PARAM;
    }

    return IR_OK;
}

static uint16_t ir_get_mapped_sample(const IR_Array_t *ir, uint8_t sensor_index)
{
    IR_ChannelMap_t map = ir->sensor_map[sensor_index];

    if ((map.adc == IR_ADC_1) && (map.rank_index < ir->adc1_len))
    {
        return ir->adc1_buf[map.rank_index];
    }

    if ((map.adc == IR_ADC_2) && (map.rank_index < ir->adc2_len))
    {
        return ir->adc2_buf[map.rank_index];
    }

    return 0u;
}

static void ir_fault(IR_Array_t *ir, IR_Status_t error)
{
    IR_Array_AllEmittersOff(ir);
    ir_timer_stop(ir);
    ir_stop_adc(ir);

    ir->last_error = error;
    ir->state = IR_STATE_FAULT;
    ir->busy = false;

    if (error == IR_TIMEOUT)
    {
        ir->timeout_count++;
    }
}

static IR_Status_t ir_validate_init(const IR_Array_Init_t *init)
{
    if (init == NULL)
    {
        return IR_BAD_PARAM;
    }

    if (init->htim_us == NULL)
    {
        return IR_BAD_PARAM;
    }

    if ((init->hadc1 == NULL) && (init->hadc2 == NULL))
    {
        return IR_BAD_PARAM;
    }

    if ((init->demux_a0.port == NULL) ||
        (init->demux_a1.port == NULL) ||
        (init->demux_a2.port == NULL) ||
        (init->demux_en.port == NULL))
    {
        return IR_BAD_PARAM;
    }

    for (uint8_t i = 0u; i < IR_ARRAY_SENSOR_COUNT; i++)
    {
        IR_ChannelMap_t map = init->sensor_map[i];

        if (map.adc == IR_ADC_1)
        {
            if ((init->hadc1 == NULL) ||
                (init->adc1_buf == NULL) ||
                (map.rank_index >= init->adc1_len))
            {
                return IR_BAD_PARAM;
            }
        }
        else if (map.adc == IR_ADC_2)
        {
            if ((init->hadc2 == NULL) ||
                (init->adc2_buf == NULL) ||
                (map.rank_index >= init->adc2_len))
            {
                return IR_BAD_PARAM;
            }
        }
        else
        {
            return IR_BAD_PARAM;
        }
    }

    return IR_OK;
}

IR_Status_t IR_Array_Init(IR_Array_t *ir, const IR_Array_Init_t *init)
{
    IR_Status_t status;

    if (ir == NULL)
    {
        return IR_BAD_PARAM;
    }

    status = ir_validate_init(init);
    if (status != IR_OK)
    {
        return status;
    }

    memset(ir, 0, sizeof(*ir));

    ir->hadc1 = init->hadc1;
    ir->hadc2 = init->hadc2;
    ir->htim_us = init->htim_us;

    ir->demux_a0 = init->demux_a0;
    ir->demux_a1 = init->demux_a1;
    ir->demux_a2 = init->demux_a2;
    ir->demux_en = init->demux_en;

    ir->demux_en_active_high = init->demux_en_active_high;

    ir->adc1_buf = init->adc1_buf;
    ir->adc1_len = init->adc1_len;

    ir->adc2_buf = init->adc2_buf;
    ir->adc2_len = init->adc2_len;

    for (uint8_t i = 0u; i < IR_ARRAY_SENSOR_COUNT; i++)
    {
        ir->sensor_map[i] = init->sensor_map[i];
    }

    ir->pulse_settle_us   = (init->pulse_settle_us > 0u)   ? init->pulse_settle_us   : 50u;
    ir->max_pulse_us      = (init->max_pulse_us > 0u)      ? init->max_pulse_us      : 200u;
    ir->adc_timeout_us    = (init->adc_timeout_us > 0u)    ? init->adc_timeout_us    : 1000u;

    if (ir->max_pulse_us <= ir->pulse_settle_us)
    {
        return IR_BAD_PARAM;
    }

    ir->frame_done_cb = init->frame_done_cb;

    ir->state = IR_STATE_IDLE;
    ir->last_error = IR_OK;
    ir->busy = false;
    ir->frame_ready = false;

    IR_Array_AllEmittersOff(ir);

    return IR_OK;
}

IR_Status_t IR_Array_StartFrame(IR_Array_t *ir)
{
    IR_Status_t status;

    if (ir == NULL)
    {
        return IR_BAD_PARAM;
    }

    if (ir->busy)
    {
        return IR_BUSY;
    }

    IR_Array_AllEmittersOff(ir);
    ir_stop_adc(ir);
    ir_timer_stop(ir);

    ir->current_sensor = 0u;
    ir->frame_ready = false;
    ir->busy = true;
    ir->last_error = IR_OK;

    /*
     * Immediately enable first LED.
     */
    ir_select_and_enable_led(ir, ir->current_sensor);

    ir->state = IR_STATE_PULSE_SETTLE;

    status = ir_timer_start_us(ir, ir->pulse_settle_us);

    if (status != IR_OK)
    {
        ir_fault(ir, status);
        return status;
    }

    return IR_OK;
}

void IR_Array_Stop(IR_Array_t *ir)
{
    if (ir == NULL)
    {
        return;
    }

    IR_Array_AllEmittersOff(ir);
    ir_timer_stop(ir);
    ir_stop_adc(ir);

    ir->busy = false;
    ir->state = IR_STATE_IDLE;
}

void IR_Array_OnTimerElapsed(IR_Array_t *ir, TIM_HandleTypeDef *htim)
{
    IR_Status_t status;

    if ((ir == NULL) || (htim != ir->htim_us))
    {
        return;
    }

    /*
     * Software one-shot timer.
     */
    ir_timer_stop(ir);

    switch (ir->state)
    {
        case IR_STATE_PULSE_SETTLE:
        {
            uint16_t remaining_us;

            ir->state = IR_STATE_LIT_ADC;

            status = ir_start_adc(ir);
            if (status != IR_OK)
            {
                ir_fault(ir, status);
                return;
            }

            /*
             * Safety timeout: if ADC/DMA hangs, this forces LED off.
             */
            remaining_us = ir->max_pulse_us - ir->pulse_settle_us;
            if (remaining_us == 0u)
            {
                remaining_us = 1u;
            }

            status = ir_timer_start_us(ir, remaining_us);
            if (status != IR_OK)
            {
                ir_fault(ir, status);
            }
            break;
        }

        case IR_STATE_LIT_ADC:
            /*
             * Timeout while LED may be on. Force everything off.
             */
            ir_fault(ir, IR_TIMEOUT);
            break;

        default:
            IR_Array_AllEmittersOff(ir);
            break;
    }
}

void IR_Array_OnAdcConvCplt(IR_Array_t *ir, ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    if ((ir == NULL) || (hadc == NULL))
    {
        return;
    }

    if (!ir->busy)
    {
        return;
    }

    if (hadc == ir->hadc1)
    {
        ir->adc_done_mask |= IR_ADC1_MASK;
    }
    else if (hadc == ir->hadc2)
    {
        ir->adc_done_mask |= IR_ADC2_MASK;
    }
    else
    {
        return;
    }

    if ((ir->adc_done_mask & ir->adc_needed_mask) != ir->adc_needed_mask)
    {
        return;
    }

    /*
     * Both ADC DMA transfers are complete.
     */
    ir_timer_stop(ir);
    ir_stop_adc(ir);

    /*
     * Turn off immediately before doing anything else.
     */
    IR_Array_AllEmittersOff(ir);

    i = ir->current_sensor;

    if (i >= IR_ARRAY_SENSOR_COUNT)
    {
        ir_fault(ir, IR_BAD_PARAM);
        return;
    }

    if (ir->state == IR_STATE_LIT_ADC)
    {
        /*
         * Raw ADC value with this LED on.
         * No ambient subtraction.
         */
        ir->signal[i] = 4095 - (int32_t)ir_get_mapped_sample(ir, i);

        ir->current_sensor++;

        if (ir->current_sensor >= IR_ARRAY_SENSOR_COUNT)
        {
            ir->busy = false;
            ir->frame_ready = true;
            ir->state = IR_STATE_DONE;
            ir->last_error = IR_OK;

            if (ir->frame_done_cb != NULL)
            {
                ir->frame_done_cb(ir);
            }
        }
        else
        {
            /*
             * Enable next LED and repeat.
             */
            ir_select_and_enable_led(ir, ir->current_sensor);

            ir->state = IR_STATE_PULSE_SETTLE;

            if (ir_timer_start_us(ir, ir->pulse_settle_us) != IR_OK)
            {
                ir_fault(ir, IR_HAL_ERROR);
            }
        }
    }
    else
    {
        IR_Array_AllEmittersOff(ir);
    }
}

bool IR_Array_IsBusy(const IR_Array_t *ir)
{
    return (ir != NULL) ? ir->busy : false;
}

bool IR_Array_FrameReady(const IR_Array_t *ir)
{
    return (ir != NULL) ? ir->frame_ready : false;
}

void IR_Array_ClearFrameReady(IR_Array_t *ir)
{
    if (ir != NULL)
    {
        ir->frame_ready = false;
    }
}
