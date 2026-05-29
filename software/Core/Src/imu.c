/*
 * imu.c
 *
 * Created on: May 27, 2026.
 * Author: david
 */

#include "imu.h"

static void IMU_CS_Select(const IMU *imu)
{
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
}

static void IMU_CS_Deselect(const IMU *imu)
{
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
}

static void IMU_WriteRegister(const IMU *imu, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = value;

    IMU_CS_Select(imu);
    HAL_SPI_Transmit(imu->hspi, tx, 2, IMU_SPI_TIMEOUT_MS);
    IMU_CS_Deselect(imu);
}

static void IMU_ReadRegisters(const IMU *imu, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t addr = reg | 0x80;

    IMU_CS_Select(imu);
    HAL_SPI_Transmit(imu->hspi, &addr, 1, IMU_SPI_TIMEOUT_MS);
    HAL_SPI_Receive(imu->hspi, data, len, IMU_SPI_TIMEOUT_MS);
    IMU_CS_Deselect(imu);
}

IMU_Status IMU_Init(IMU *imu, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, IMU_Gyro_ODR odr_mode, IMU_LPF1_State lpf1_state)
{
    if (imu == NULL || hspi == NULL || cs_port == NULL)
        return IMU_ERROR;

    imu->hspi = hspi;
    imu->cs_port = cs_port;
    imu->cs_pin = cs_pin;
    imu->odr_mode = odr_mode;
    imu->lpf1_state = lpf1_state;
    imu->raw_gyro_z = 0;
    imu->gyro_z_dps = 0.0f;
    imu->gyro_z_rad_s = 0.0f;
    imu->gyro_z_bias = 0.0f;

    imu->data_ready = 0;

    IMU_CS_Deselect(imu);
    HAL_Delay(100);

    uint8_t who_am_i = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        IMU_ReadRegisters(imu, LSM6DSO32_REG_WHO_AM_I, &who_am_i, 1);

        if (who_am_i == LSM6DSO32_WHO_AM_I_VAL)
        {
            break;
        }

        HAL_Delay(5);
    } IMU_ReadRegisters(imu, LSM6DSO32_REG_WHO_AM_I, &who_am_i, 1);

    if (who_am_i != LSM6DSO32_WHO_AM_I_VAL)
    {
        return IMU_WRONG_DEVICE;
    }

    // CTRL3_C (BDU=1, IF_INC=1)
    IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL3_C, 0x44);

    // Turn off accelerometer
    IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL1_XL, 0x00);

    // Config CTRL4_C for LPF1 on gyro
    if (imu->lpf1_state == IMU_LPF1_ENABLED)
    {
    	// Set bit 1 (LPF1_SEL_G) to 1 to activate LPF1
        IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL4_C, 0x02);
        IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL6_C, 0x02);
    }
    else
    {
        // LPF1 (bypassed)
        IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL4_C, 0x00);
    }

    // Config gyro: ODR (first 4 bits) + Full Scale at +-1000 dps (0x08
    uint8_t ctrl2_g_value = (uint8_t)imu->odr_mode | 0x08;
    IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL2_G, ctrl2_g_value);

    IMU_WriteRegister(imu, LSM6DSO32_REG_CTRL7_G, 0x00);

    IMU_WriteRegister(imu, LSM6DSO32_REG_INT2_CTRL, LSM6DSO32_INT2_DRDY_G);

    HAL_Delay(50);

    return IMU_OK;
}

IMU_Status IMU_CalibrateGyro(IMU *imu, uint16_t sample_count)
{
    if (imu == NULL)
        return IMU_ERROR;

    int32_t gyro_z_sum = 0;
    uint8_t data[2];

    for (uint16_t i = 0; i < sample_count; i++)
    {
        IMU_ReadRegisters(imu, LSM6DSO32_REG_OUTZ_L_G, data, 2);
        int16_t z_raw = (int16_t)((data[1] << 8) | data[0]);

        gyro_z_sum += z_raw;
        HAL_Delay(1);
    }

    if (sample_count > 0)
    {
        imu->gyro_z_bias = (float)gyro_z_sum / (float)sample_count;
        return IMU_OK;
    }

    return IMU_ERROR;
}

IMU_Status IMU_Update(IMU *imu)
{
    if (imu == NULL)
        return IMU_ERROR;

    uint8_t data[2];

    IMU_ReadRegisters(imu, LSM6DSO32_REG_OUTZ_L_G, data, 2);

    // negative because imu is on the bottom of the pcb
    imu->raw_gyro_z = (int16_t)((data[1] << 8) | data[0]);

    float corrected_gyro = (float)imu->raw_gyro_z - imu->gyro_z_bias;
    imu->gyro_z_dps = corrected_gyro * LSM6DSO32_GYRO_SENS_1000;
    imu->gyro_z_rad_s = imu->gyro_z_dps * IMU_PI_F / 180.0f;

    return IMU_OK;
}

void IMU_NotifyDataReady(IMU *imu)
{
    if (imu == NULL)
        return;

    imu->data_ready = 1;
}

uint8_t IMU_UpdateIfReady(IMU *imu)
{
    if (imu == NULL)
        return 0;

    if (imu->data_ready == 0)
        return 0;

    imu->data_ready = 0;

    return (IMU_Update(imu) == IMU_OK) ? 1 : 0;
}

float IMU_GetGyroZDeg(const IMU *imu)
{
    if (imu == NULL)
        return 0.0f;


    return GYRO_Z_ANGLE_DIR * imu->gyro_z_dps;
}

float IMU_GetGyroZRad(const IMU *imu)
{
    if (imu == NULL)
        return 0.0f;

    return GYRO_Z_ANGLE_DIR*imu->gyro_z_rad_s;
}
