/*
 * ir_sensors.c
 *
 *  Created on: May 28, 2026
 *      Author: andrija
 */

#include "ir_sensors.h"

/*
    CubeMX ADC order:

    ADC1 DMA buffer:
        adc1_buf[0] = IR_REC_2
        adc1_buf[1] = IR_REC_3
        adc1_buf[2] = IR_REC_4
        adc1_buf[3] = IR_REC_5

    ADC2 DMA buffer:
        adc2_buf[0] = IR_REC_1
        adc2_buf[1] = IR_REC_6
*/

#define IR_ADC1_NUM_CHANNELS 4
#define IR_ADC2_NUM_CHANNELS 2

#define IR_LED_SETTLE_US     20u
#define IR_BETWEEN_PULSES_US 50u

typedef enum
{
    IR_ADC_BUS_1 = 0,
    IR_ADC_BUS_2
} IR_AdcBus_t;

typedef struct
{
    IR_AdcBus_t adc_bus;
    uint8_t buffer_index;
} IR_AdcMap_t;

typedef enum
{
    IR_STATE_IDLE = 0,
    IR_STATE_START_AMBIENT_DMA,
    IR_STATE_WAIT_AMBIENT_DMA,
    IR_STATE_LED_SETTLE,
    IR_STATE_START_ACTIVE_DMA,
    IR_STATE_WAIT_ACTIVE_DMA,
    IR_STATE_GAP
} IR_State_t;

static ADC_HandleTypeDef *hadc1 = NULL;
static ADC_HandleTypeDef *hadc2 = NULL;

static uint16_t adc1_buf[IR_ADC1_NUM_CHANNELS];
static uint16_t adc2_buf[IR_ADC2_NUM_CHANNELS];

static volatile bool adc1_done = false;
static volatile bool adc2_done = false;

static IR_State_t state = IR_STATE_IDLE;
static IR_Data_t ir_data;

static uint8_t current_sensor = 0;
static uint32_t deadline_us = 0;
static uint32_t cycles_per_us = 1;

/*
    Sensor index:
        0 = IR_REC_1
        1 = IR_REC_2
        2 = IR_REC_3
        3 = IR_REC_4
        4 = IR_REC_5
        5 = IR_REC_6
*/
static const IR_AdcMap_t ir_adc_map[IR_NUM_SENSORS] =
{
    {IR_ADC_BUS_2, 0},   // IR_REC_1
    {IR_ADC_BUS_1, 0},   // IR_REC_2
    {IR_ADC_BUS_1, 1},   // IR_REC_3
    {IR_ADC_BUS_1, 2},   // IR_REC_4
    {IR_ADC_BUS_1, 3},   // IR_REC_5
    {IR_ADC_BUS_2, 1}    // IR_REC_6
};

static void IR_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000u;
    if (cycles_per_us == 0u)
    {
        cycles_per_us = 1u;
    }
}

static uint32_t IR_Micros(void)
{
    return DWT->CYCCNT / cycles_per_us;
}

static bool IR_TimeElapsed(uint32_t deadline)
{
    return ((int32_t)(IR_Micros() - deadline) >= 0);
}

static void IR_AllEmittersOff(void)
{
    HAL_GPIO_WritePin(DMUX_EN_GPIO_Port, DMUX_EN_Pin, GPIO_PIN_RESET);
}

static void IR_SelectEmitter(uint8_t emitter_index)
{
    /*
        emitter_index:
            0 -> IR_EM_1 / Y0
            1 -> IR_EM_2 / Y1
            ...
            5 -> IR_EM_6 / Y5
    */

    HAL_GPIO_WritePin(DMUX_A0_GPIO_Port, DMUX_A0_Pin,
                      (emitter_index & 0x01u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DMUX_A1_GPIO_Port, DMUX_A1_Pin,
                      (emitter_index & 0x02u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(DMUX_A2_GPIO_Port, DMUX_A2_Pin,
                      (emitter_index & 0x04u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /*
        In your schematic the active-high enable input of the 74AHC238 is controlled
        by DMUX_EN, while the active-low enables are tied low.
    */
    HAL_GPIO_WritePin(DMUX_EN_GPIO_Port, DMUX_EN_Pin, GPIO_PIN_SET);
}

static uint16_t IR_ReadMappedSensor(uint8_t sensor_index)
{
    IR_AdcMap_t map = ir_adc_map[sensor_index];

    if (map.adc_bus == IR_ADC_BUS_1)
    {
        return adc1_buf[map.buffer_index];
    }
    else
    {
        return adc2_buf[map.buffer_index];
    }
}

static void IR_ResetDmaDoneFlags(void)
{
    adc1_done = false;
    adc2_done = false;
}

static bool IR_AreAdcsDone(void)
{
    return adc1_done && adc2_done;
}

static HAL_StatusTypeDef IR_StartAdcDma(void)
{
    HAL_StatusTypeDef status1;
    HAL_StatusTypeDef status2;

    IR_ResetDmaDoneFlags();

    status1 = HAL_ADC_Start_DMA(hadc1, (uint32_t*)adc1_buf, IR_ADC1_NUM_CHANNELS);
    status2 = HAL_ADC_Start_DMA(hadc2, (uint32_t*)adc2_buf, IR_ADC2_NUM_CHANNELS);

    if ((status1 != HAL_OK) || (status2 != HAL_OK))
    {
        IR_AllEmittersOff();
        state = IR_STATE_IDLE;
        return HAL_ERROR;
    }

    return HAL_OK;
}

void IR_Init(ADC_HandleTypeDef *hadc1_, ADC_HandleTypeDef *hadc2_)
{
    hadc1 = hadc1_;
    hadc2 = hadc2_;

    IR_DWT_Init();

    IR_AllEmittersOff();

    for (uint8_t i = 0; i < IR_NUM_SENSORS; i++)
    {
        ir_data.ambient[i] = 0;
        ir_data.active[i] = 0;
        ir_data.signal[i] = 0;
    }

    ir_data.scan_ready = false;
    ir_data.scan_count = 0;

    state = IR_STATE_IDLE;
}

bool IR_StartScan(void)
{
    if (state != IR_STATE_IDLE)
    {
        return false;
    }

    ir_data.scan_ready = false;
    current_sensor = 0;

    IR_AllEmittersOff();

    state = IR_STATE_START_AMBIENT_DMA;

    return true;
}

void IR_Task(void)
{
    switch (state)
    {
        case IR_STATE_IDLE:
        {
            break;
        }

        case IR_STATE_START_AMBIENT_DMA:
        {
            IR_AllEmittersOff();

            if (IR_StartAdcDma() == HAL_OK)
            {
                state = IR_STATE_WAIT_AMBIENT_DMA;
            }

            break;
        }

        case IR_STATE_WAIT_AMBIENT_DMA:
        {
            if (IR_AreAdcsDone())
            {
                ir_data.ambient[current_sensor] = IR_ReadMappedSensor(current_sensor);

                IR_SelectEmitter(current_sensor);

                deadline_us = IR_Micros() + IR_LED_SETTLE_US;
                state = IR_STATE_LED_SETTLE;
            }

            break;
        }

        case IR_STATE_LED_SETTLE:
        {
            if (IR_TimeElapsed(deadline_us))
            {
                if (IR_StartAdcDma() == HAL_OK)
                {
                    state = IR_STATE_WAIT_ACTIVE_DMA;
                }
            }

            break;
        }

        case IR_STATE_WAIT_ACTIVE_DMA:
        {
            if (IR_AreAdcsDone())
            {
                IR_AllEmittersOff();

                ir_data.active[current_sensor] = IR_ReadMappedSensor(current_sensor);

                /*
  	  	  	  	  ADC near max means no obstacle, low ADC means obstacle so we invert it
                */
                ir_data.signal[current_sensor] =
                    (int16_t)ir_data.ambient[current_sensor] -
                    (int16_t)ir_data.active[current_sensor];

                deadline_us = IR_Micros() + IR_BETWEEN_PULSES_US;
                state = IR_STATE_GAP;
            }

            break;
        }

        case IR_STATE_GAP:
        {
            if (IR_TimeElapsed(deadline_us))
            {
                current_sensor++;

                if (current_sensor >= IR_NUM_SENSORS)
                {
                    ir_data.scan_ready = true;
                    ir_data.scan_count++;
                    state = IR_STATE_IDLE;
                }
                else
                {
                    state = IR_STATE_START_AMBIENT_DMA;
                }
            }

            break;
        }

        default:
        {
            IR_AllEmittersOff();
            state = IR_STATE_IDLE;
            break;
        }
    }
}

bool IR_IsBusy(void)
{
    return state != IR_STATE_IDLE;
}

bool IR_IsScanReady(void)
{
    return ir_data.scan_ready;
}

void IR_ClearScanReadyFlag(void)
{
    ir_data.scan_ready = false;
}

const IR_Data_t* IR_GetData(void)
{
    return &ir_data;
}

void IR_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == hadc1)
    {
        adc1_done = true;
    }
    else if (hadc == hadc2)
    {
        adc2_done = true;
    }
}
