/*
 * app.c
 *
 *  Created on: May 29, 2026.
 *      Author: david
 */

#include "app.h"
#include "main.h"

#include "params.h"

#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "encoder.h"
#include "neopixel.h"
#include "imu.h"
#include "velocity_pi.h"
#include "button.h"
#include "robot_controller.h"
#include "ir_sensors.h"
#include "motion.h"

// Defined in main.c because of MX
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern SPI_HandleTypeDef  hspi1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;

/* ------------------------------------------------------------------------- */
/* Application objects                                                       */
/* ------------------------------------------------------------------------- */

Motor motor_left =
{
    .htim_in1 = &htim1,
    .channel_in1 = TIM_CHANNEL_1,

    .htim_in2 = &htim1,
    .channel_in2 = TIM_CHANNEL_2,

    .direction = PARAM_MOTOR_DIR_LEFT
};

Motor motor_right =
{
    .htim_in1 = &htim1,
    .channel_in1 = TIM_CHANNEL_3,

    .htim_in2 = &htim1,
    .channel_in2 = TIM_CHANNEL_4,

    .direction = PARAM_MOTOR_DIR_RIGHT
};

Encoder enc_left;
Encoder enc_right;

NeoPixel neopixel;

IMU imu;
float yaw;

VelocityPI pi_left;
VelocityPI pi_right;
float dt;

Button btn1;
Button btn2;
DipSwitch dip_sw;

IR_DemuxPins_t ir_demux = {
    .a0 = {DMUX_A0_GPIO_Port, DMUX_A0_Pin},
    .a1 = {DMUX_A1_GPIO_Port, DMUX_A1_Pin},
    .a2 = {DMUX_A2_GPIO_Port, DMUX_A2_Pin},
    .en = {DMUX_EN_GPIO_Port, DMUX_EN_Pin},

    .en_active_high = true,
};

RobotController robot;

static MotionController motion;
static MotionCommand motion_cmd;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static float Timer_GetUpdatePeriod_s(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clk_hz;

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1)
        timer_clk_hz = pclk1;
    else
        timer_clk_hz = 2U * pclk1;

    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = htim->Instance->ARR;

    float update_freq_hz =
        (float)timer_clk_hz / ((float)(psc + 1U) * (float)(arr + 1U));

    return 1.0f / update_freq_hz;
}

void _integrate_omega(float omega, float dt_s, float *yaw_out)
{
    *yaw_out += omega * dt_s;
}

static void App_GetMotionFeedback(MotionFeedback *fb)
{
    fb->x_m = RobotController_GetPoseX(&robot);
    fb->y_m = RobotController_GetPoseY(&robot);

    fb->yaw_rad = yaw;

    fb->linear_speed_mps = RobotController_GetLinearVelocity(&robot);
    fb->angular_speed_radps = IMU_GetGyroZRad(&imu);
}

/* ------------------------------------------------------------------------- */
/* HAL interrupt callbacks                                                   */
/* ------------------------------------------------------------------------- */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
        NeoPixel_DMA_Callback(&neopixel, htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == btn1.pin)
        Button_EXTI_Callback(&btn1, GPIO_Pin, &htim7);
    else if (GPIO_Pin == btn2.pin)
        Button_EXTI_Callback(&btn2, GPIO_Pin, &htim7);

    if (GPIO_Pin == IMU_INT2_Pin)
        IMU_NotifyDataReady(&imu);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
        if (!IR_Sensors_IsBusy() && !IR_Sensors_FrameReady())
            IR_Sensors_StartFrame();

    if (htim->Instance == TIM6)
    {
        _integrate_omega(IMU_GetGyroZRad(&imu), dt, &yaw);

        MotionFeedback fb;
        App_GetMotionFeedback(&fb);
        Motion_Update(&motion, &fb, &motion_cmd);

        if (motion_cmd.active)
            RobotController_SetCmdVel(&robot, motion_cmd.linear_mps, motion_cmd.angular_radps);
        else
            RobotController_SetCmdVel(&robot, 0.0f, 0.0f);

        RobotController_Update(&robot, dt);
    }

    if (htim->Instance == TIM7)
        Button_TIM_PeriodElapsedCallback(htim);

    if (htim->Instance == TIM16)
        IR_Sensors_OnTimerElapsed(htim);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    IR_Sensors_OnAdcConvCplt(hadc);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void App_Init(void)
{
    dt = Timer_GetUpdatePeriod_s(&htim6);

    Motor_Init(&motor_left);
    Motor_Init(&motor_right);

    Encoder_Init(&enc_left,  &htim2, PARAM_ENC_TICKS_PER_REV, PARAM_ENC_DIR_LEFT, PARAM_ENC_VEL_FILTER_TAU_S);
    Encoder_Init(&enc_right, &htim4, PARAM_ENC_TICKS_PER_REV,  PARAM_ENC_DIR_RIGHT, PARAM_ENC_VEL_FILTER_TAU_S);

    VelocityPI_Init(&pi_left,
                    PARAM_PI_KP,
					PARAM_PI_KI,
                    dt,
                    -PARAM_PI_MAX_PWM_OUT,
					PARAM_PI_MAX_PWM_OUT);

    VelocityPI_Init(&pi_right,
    				PARAM_PI_KP,
					PARAM_PI_KI,
                    dt,
                    -PARAM_PI_MAX_PWM_OUT,
					PARAM_PI_MAX_PWM_OUT);

    VelocityPI_SetSetpoint(&pi_left,  0.0f);
    VelocityPI_SetSetpoint(&pi_right, 0.0f);

    NeoPixel_Init(&neopixel, &htim8, TIM_CHANNEL_1);
    NeoPixel_Off(&neopixel);

    Button_Init(&btn1, BTN_1_GPIO_Port, BTN_1_Pin);
    Button_Init(&btn2, BTN_2_GPIO_Port, BTN_2_Pin);

    DipSwitch_Init(&dip_sw,
                   DIP_1_GPIO_Port, DIP_1_Pin,
                   DIP_2_GPIO_Port, DIP_2_Pin,
                   DIP_3_GPIO_Port, DIP_3_Pin,
                   GPIO_PIN_SET);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    IR_Sensors_Init(&hadc1, &hadc2, &htim16, &ir_demux);

    RobotController_Init(&robot,
                         &motor_left, &motor_right,
                         &enc_left,   &enc_right,
                         &pi_left,    &pi_right);

    IMU_Status imu_status;

    imu_status = IMU_Init(
            &imu,
            &hspi1,
            IMU_CS_GPIO_Port,
            IMU_CS_Pin,
            IMU_GYRO_ODR_1_66_KHZ,
            IMU_LPF1_ENABLED);

    if (imu_status == IMU_OK)
        NeoPixel_SetColor(&neopixel, COLOR_GREEN);
    else
        NeoPixel_SetColor(&neopixel, COLOR_RED);

    NeoPixel_Show(&neopixel);

    /* IMU calibration */
    HAL_Delay(PARAM_IMU_CALIB_SETTLE_MS);

    NeoPixel_SetColor(&neopixel, COLOR_BLUE);
    NeoPixel_Show(&neopixel);

    if (imu_status == IMU_OK)
        IMU_CalibrateGyro(&imu, PARAM_IMU_CALIB_SAMPLES);

    NeoPixel_Off(&neopixel);

    IMU_Update(&imu);

    MotionConfig cfg;

    Motion_DefaultConfig(&cfg);

    cfg.control_dt_s = dt;   /* 100 Hz TIM6 loop */

    cfg.max_linear_speed_mps = PARAM_MOTION_MAX_LINEAR_SPEED_MPS;
    cfg.max_linear_accel_mps2 = PARAM_MOTION_MAX_LINEAR_ACCEL_MPS2;

    cfg.max_angular_speed_radps = PARAM_MOTION_MAX_ANGULAR_SPEED_RADPS;
    cfg.max_angular_accel_radps2 = PARAM_MOTION_MAX_ANGULAR_ACCEL_RADPS2;

    cfg.distance_kp = PARAM_MOTION_DISTANCE_KP;
    cfg.heading_kp = PARAM_MOTION_HEADING_KP;
    cfg.turn_kp = PARAM_MOTION_TURN_KP;

    cfg.use_lateral_odometry_correction = PARAM_MOTION_USE_LATERAL_CORR;
    cfg.lateral_kp = PARAM_MOTION_LATERAL_KP;

    Motion_Init(&motion, &cfg);

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim6);
}

void App_Loop(void)
{
	if (IR_Sensors_FrameReady())
	{
		const int32_t *ir = IR_Sensors_GetSignal();

		int32_t ir1 = ir[0];
		int32_t ir2 = ir[1];
		int32_t ir3 = ir[2];
		int32_t ir4 = ir[3];
		int32_t ir5 = ir[4];
		int32_t ir6 = ir[5];

		IR_Sensors_ClearFrameReady();
	}
}
