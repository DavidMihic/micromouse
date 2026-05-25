/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIP_1_Pin GPIO_PIN_13
#define DIP_1_GPIO_Port GPIOC
#define DIP_2_Pin GPIO_PIN_14
#define DIP_2_GPIO_Port GPIOC
#define DIP_3_Pin GPIO_PIN_15
#define DIP_3_GPIO_Port GPIOC
#define E_A_L_Pin GPIO_PIN_0
#define E_A_L_GPIO_Port GPIOA
#define E_B_L_Pin GPIO_PIN_1
#define E_B_L_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define IMU_INT2_Pin GPIO_PIN_0
#define IMU_INT2_GPIO_Port GPIOB
#define IR_REC_2_Pin GPIO_PIN_1
#define IR_REC_2_GPIO_Port GPIOB
#define IR_REC_1_Pin GPIO_PIN_2
#define IR_REC_1_GPIO_Port GPIOB
#define DMUX_EN_Pin GPIO_PIN_10
#define DMUX_EN_GPIO_Port GPIOB
#define IR_REC_3_Pin GPIO_PIN_11
#define IR_REC_3_GPIO_Port GPIOB
#define IR_REC_4_Pin GPIO_PIN_12
#define IR_REC_4_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_13
#define BTN_2_GPIO_Port GPIOB
#define IR_REC_5_Pin GPIO_PIN_14
#define IR_REC_5_GPIO_Port GPIOB
#define IR_REC_6_Pin GPIO_PIN_15
#define IR_REC_6_GPIO_Port GPIOB
#define IN_1_L_Pin GPIO_PIN_8
#define IN_1_L_GPIO_Port GPIOA
#define IN_2_L_Pin GPIO_PIN_9
#define IN_2_L_GPIO_Port GPIOA
#define IN_1_R_Pin GPIO_PIN_10
#define IN_1_R_GPIO_Port GPIOA
#define IN_2_R_Pin GPIO_PIN_11
#define IN_2_R_GPIO_Port GPIOA
#define E_B_R_Pin GPIO_PIN_12
#define E_B_R_GPIO_Port GPIOA
#define LED_IN_Pin GPIO_PIN_15
#define LED_IN_GPIO_Port GPIOA
#define BTN_1_Pin GPIO_PIN_4
#define BTN_1_GPIO_Port GPIOB
#define DMUX_A0_Pin GPIO_PIN_5
#define DMUX_A0_GPIO_Port GPIOB
#define E_A_R_Pin GPIO_PIN_6
#define E_A_R_GPIO_Port GPIOB
#define DMUX_A1_Pin GPIO_PIN_7
#define DMUX_A1_GPIO_Port GPIOB
#define DMUX_A2_Pin GPIO_PIN_9
#define DMUX_A2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
