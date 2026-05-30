/*
 * imu.h
 *
 * Created on: May 27, 2026.
 * Author: david
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* LSM6DSO32 Registri */
#define LSM6DSO32_REG_WHO_AM_I  0x0F
#define LSM6DSO32_REG_CTRL1_XL  0x10
#define LSM6DSO32_REG_CTRL2_G   0x11
#define LSM6DSO32_REG_CTRL3_C   0x12
#define LSM6DSO32_REG_CTRL4_C   0x13
#define LSM6DSO32_REG_CTRL6_C   0x15
#define LSM6DSO32_REG_CTRL7_G   0x16
#define LSM6DSO32_REG_OUTZ_L_G  0x26
#define LSM6DSO32_REG_OUTZ_H_G  0x27
#define LSM6DSO32_REG_INT2_CTRL   0x0E
#define LSM6DSO32_INT2_DRDY_G     0x02

#define LSM6DSO32_WHO_AM_I_VAL  0x6C

// Sensitivity factor for +-1000 dps is 35.00 mdps/LSB (datasheet)
// Divide by 1000 to get dps = 0.035f
#define LSM6DSO32_GYRO_SENS_1000 0.035f
#define IMU_PI_F 3.14159265358979323846f
#define IMU_SPI_TIMEOUT_MS 2

// IMU is on the bottom of the PCB so its -1
#define GYRO_Z_ANGLE_DIR -1

typedef enum
{
    IMU_OK = 0,
    IMU_ERROR,
    IMU_WRONG_DEVICE
} IMU_Status;

typedef enum
{
    IMU_GYRO_ODR_OFF      = 0x00,
    IMU_GYRO_ODR_12_5_HZ = 0x10,
    IMU_GYRO_ODR_26_HZ   = 0x20,
    IMU_GYRO_ODR_52_HZ   = 0x30,
    IMU_GYRO_ODR_104_HZ  = 0x40,
    IMU_GYRO_ODR_208_HZ  = 0x50,
    IMU_GYRO_ODR_416_HZ  = 0x60,
    IMU_GYRO_ODR_833_HZ  = 0x70,
    IMU_GYRO_ODR_1_66_KHZ= 0x80,
    IMU_GYRO_ODR_3_33_KHZ= 0x90,
    IMU_GYRO_ODR_6_66_KHZ= 0xA0
} IMU_Gyro_ODR;

typedef enum
{
    IMU_LPF1_DISABLED = 0,
    IMU_LPF1_ENABLED
} IMU_LPF1_State;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    IMU_Gyro_ODR odr_mode;
    IMU_LPF1_State lpf1_state;

    volatile uint8_t data_ready;

    int16_t raw_gyro_z;
    float gyro_z_dps;
    float gyro_z_rad_s;
    float gyro_z_bias;
} IMU;

IMU_Status IMU_Init(IMU *imu, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, IMU_Gyro_ODR odr_mode, IMU_LPF1_State lpf1_state);
IMU_Status IMU_CalibrateGyro(IMU *imu, uint16_t sample_count);
IMU_Status IMU_Update(IMU *imu);

void IMU_NotifyDataReady(IMU *imu);
uint8_t IMU_UpdateIfReady(IMU *imu);

float IMU_GetGyroZDeg(const IMU *imu);
float IMU_GetGyroZRad(const IMU *imu);

#endif /* INC_IMU_H_ */
