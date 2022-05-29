/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Hz100 99
#define Hz10 999
#define Pre_20k_360 9
#define Counter_20kHz_360 359
#define Pre_10kHz 7199
#define UART_SPEED 115200
#define Hz1000 9
#define Hz20 499
#define STOP_AW_Pin GPIO_PIN_1
#define STOP_AW_GPIO_Port GPIOF
#define ENCODER_A_R_Pin GPIO_PIN_0
#define ENCODER_A_R_GPIO_Port GPIOA
#define ENCODER_B_R_Pin GPIO_PIN_1
#define ENCODER_B_R_GPIO_Port GPIOA
#define DIR1_1_Pin GPIO_PIN_2
#define DIR1_1_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_3
#define ADC1_GPIO_Port GPIOA
#define SERWO_OUT_Pin GPIO_PIN_4
#define SERWO_OUT_GPIO_Port GPIOA
#define KRANCOWKA_1_Pin GPIO_PIN_5
#define KRANCOWKA_1_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_6
#define PWM_1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_7
#define ADC2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_0
#define PWM_3_GPIO_Port GPIOB
#define PWM_4_Pin GPIO_PIN_1
#define PWM_4_GPIO_Port GPIOB
#define ENCODER_A_L_Pin GPIO_PIN_8
#define ENCODER_A_L_GPIO_Port GPIOA
#define ENCODER_B_L_Pin GPIO_PIN_9
#define ENCODER_B_L_GPIO_Port GPIOA
#define DIR2_2_Pin GPIO_PIN_10
#define DIR2_2_GPIO_Port GPIOA
#define DIR2_1_Pin GPIO_PIN_11
#define DIR2_1_GPIO_Port GPIOA
#define DIR1_2_Pin GPIO_PIN_12
#define DIR1_2_GPIO_Port GPIOA
#define PILOT_UART_RX_Pin GPIO_PIN_15
#define PILOT_UART_RX_GPIO_Port GPIOA
#define PILOT_UART_TX_Pin GPIO_PIN_3
#define PILOT_UART_TX_GPIO_Port GPIOB
#define KRANCOWKA_2_Pin GPIO_PIN_4
#define KRANCOWKA_2_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_5
#define PWM_2_GPIO_Port GPIOB
#define PC_UART_TX_Pin GPIO_PIN_6
#define PC_UART_TX_GPIO_Port GPIOB
#define PC_UART_RX_Pin GPIO_PIN_7
#define PC_UART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
