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
#define Hz20 499
#define Hz10 999
#define Pre_20k_360 19
#define UART_115 115200
#define Pre_10kHz 7199
#define Pre_10k 7199
#define UART_SPEED_MCU 921600
#define Hz1000 9
#define Counter_20kHz_360 359
#define Counter_10kHz_1 9999
#define UART_230 230400
#define STOP_AW_Pin GPIO_PIN_1
#define STOP_AW_GPIO_Port GPIOF
#define STOP_AW_EXTI_IRQn EXTI1_IRQn
#define ENCODER_A_LT_Pin GPIO_PIN_0
#define ENCODER_A_LT_GPIO_Port GPIOA
#define ENCODER_B_LT_Pin GPIO_PIN_1
#define ENCODER_B_LT_GPIO_Port GPIOA
#define PC_UART_TX_Pin GPIO_PIN_2
#define PC_UART_TX_GPIO_Port GPIOA
#define PC_UART_RX_Pin GPIO_PIN_3
#define PC_UART_RX_GPIO_Port GPIOA
#define DIR_PT_Pin GPIO_PIN_4
#define DIR_PT_GPIO_Port GPIOA
#define DIR_LT_Pin GPIO_PIN_5
#define DIR_LT_GPIO_Port GPIOA
#define ENCODER_A_PT_Pin GPIO_PIN_6
#define ENCODER_A_PT_GPIO_Port GPIOA
#define ENCODER_B_PT_Pin GPIO_PIN_7
#define ENCODER_B_PT_GPIO_Port GPIOA
#define DIR_LP_Pin GPIO_PIN_0
#define DIR_LP_GPIO_Port GPIOB
#define DIR_PP_Pin GPIO_PIN_1
#define DIR_PP_GPIO_Port GPIOB
#define PWM1_LT_Pin GPIO_PIN_8
#define PWM1_LT_GPIO_Port GPIOA
#define PWM2_PT_Pin GPIO_PIN_9
#define PWM2_PT_GPIO_Port GPIOA
#define PWM3_LP_Pin GPIO_PIN_10
#define PWM3_LP_GPIO_Port GPIOA
#define PWM4_PP_Pin GPIO_PIN_11
#define PWM4_PP_GPIO_Port GPIOA
#define DIR_SERWO_Pin GPIO_PIN_12
#define DIR_SERWO_GPIO_Port GPIOA
#define WYLACZNIK_Pin GPIO_PIN_15
#define WYLACZNIK_GPIO_Port GPIOA
#define KRANCOWKA_1_Pin GPIO_PIN_3
#define KRANCOWKA_1_GPIO_Port GPIOB
#define KRANCOWKA_1_EXTI_IRQn EXTI3_IRQn
#define KRANCOWKA_2_Pin GPIO_PIN_4
#define KRANCOWKA_2_GPIO_Port GPIOB
#define KRANCOWKA_2_EXTI_IRQn EXTI4_IRQn
#define PWM5_SERWO_Pin GPIO_PIN_5
#define PWM5_SERWO_GPIO_Port GPIOB
#define COMM_TO_MASTER_UART_TX_Pin GPIO_PIN_6
#define COMM_TO_MASTER_UART_TX_GPIO_Port GPIOB
#define COMM_TO_MASTER_UART_RX_Pin GPIO_PIN_7
#define COMM_TO_MASTER_UART_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
