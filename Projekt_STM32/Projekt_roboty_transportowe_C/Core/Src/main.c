/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//https://github.com/mokhwasomssi/stm32_hal_ibus
#include "ibus.h"
#include <stdio.h>
#include <stdlib.h>
#include "PID.h"
#include "MOT.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define IBUS_UART (&huart2)
#define IBUS_USER_CHANNELS 6

#define ENCODER_LEWY (&htim1)
#define ENCODER_PRAWY (&htim2)
#define PWM_TIMER (&htim3)
#define PID_TIMER (&htim6)
#define COMM_TIMER (&htim7)
#define COMM_HZ 10
#define PID_HZ 20

#define CW 0
#define CCW 1

#define PID_POWER 100

#define CH_TYL_LEWY                        0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define CH_TYL_PRAWY                       0x00000004U                          /*!< Capture/compare channel 2 identifier      */
#define CH_PRZOD_LEWY                      0x00000008U                          /*!< Capture/compare channel 3 identifier      */
#define CH_PRZOD_PRAWY 					   0x0000000CU

//DEFINES INSIDE MOT.h OF MOTOR PROPERTIES

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t Wiadomosc[200];
int16_t Rozmiar;
int16_t test = 10;

bool odebranie;
bool MA;

int8_t PID_flaga = 0;
int8_t COMM_flaga = 0;

uint16_t ibus_data[IBUS_USER_CHANNELS];

int16_t sterowanie_tyl_lewy = 0;
int16_t sterowanie_tyl_prawy = 0;

int16_t pomiar_tyl_lewy = 0;
int16_t pomiar_tyl_prawy = 0;

int16_t setpoint_tyl_lewy = 0;
int16_t setpoint_tyl_prawy = 0;

PID_t Pid_tyl_lewy;
PID_t Pid_tyl_prawy;

motor mot_tyl_lewy;
motor mot_tyl_prawy;
int16_t ms = 20;

int16_t I1_V;
int16_t I2_OMEGA;
int16_t I3_PLCHLDR;
int16_t I4_V_MAX;
int16_t I5_MA;
int16_t I6_ONOFF;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int16_t to_process_range(int16_t input);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //INIT komunikacji ze zdalna aparatura
  ibus_init();

  //timery dla PID i komunikacji z PC
  HAL_TIM_Base_Start_IT(PID_TIMER);
  HAL_TIM_Base_Start_IT(COMM_TIMER);

  //timer PWM i zmienna sterujaca
  /*TIM_OC_InitTypeDef oc;
  oc.OCMode = TIM_OCMODE_PWM2;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
  oc.OCFastMode = TIM_OCFAST_ENABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_SET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &oc, CH_TYL_LEWY);
  HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &oc, CH_TYL_PRAWY);
  HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &oc, CH_PRZOD_LEWY);
  HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &oc, CH_PRZOD_PRAWY);*/


  //Pid inicjalizacja
  ms = (int16_t)1000/PID_HZ;
  pid_init(&Pid_tyl_lewy, 3.0, 15.0, 0.0, ms,PID_POWER);
  pid_scaling(&Pid_tyl_lewy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);
  pid_init(&Pid_tyl_prawy, 3.0, 15.0, 0.0, ms,PID_POWER);
  pid_scaling(&Pid_tyl_prawy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);

  //Motor inicjalizacja
  motor_init(&mot_tyl_lewy,DIR1_1_GPIO_Port,DIR1_2_GPIO_Port,DIR1_1_Pin,DIR1_2_Pin);
  motor_init(&mot_tyl_prawy,DIR2_1_GPIO_Port,DIR2_2_GPIO_Port,DIR2_1_Pin,DIR2_2_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(PID_flaga == 1){
		  PID_flaga = 0;

		  pomiar_tyl_lewy = to_process_range(motor_calculate_speed(&mot_tyl_lewy, ENCODER_LEWY, PID_HZ));
		  sterowanie_tyl_lewy = pid_calc(&Pid_tyl_lewy, pomiar_tyl_lewy, setpoint_tyl_lewy);

		  if(MA == 1){
			  if (I1_V != 0){
				  set_motor_dir(&mot_tyl_lewy,sterowanie_tyl_lewy);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_LEWY,abs(sterowanie_tyl_lewy));
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_LEWY,abs(sterowanie_tyl_lewy));
			  }
			  else{
				  set_motor_dir(&mot_tyl_lewy,0);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_LEWY,0);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_LEWY,0);
			  }
		  }
		  pomiar_tyl_prawy = to_process_range(motor_calculate_speed(&mot_tyl_prawy, ENCODER_PRAWY, PID_HZ));
		  sterowanie_tyl_prawy = pid_calc(&Pid_tyl_prawy, pomiar_tyl_prawy, setpoint_tyl_prawy);

		  if(MA == 1){
			  if (I1_V != 0){
				  set_motor_dir(&mot_tyl_prawy,sterowanie_tyl_prawy);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_PRAWY,abs(sterowanie_tyl_prawy));
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_PRAWY,abs(sterowanie_tyl_prawy));
			  }
			  else{
				  set_motor_dir(&mot_tyl_prawy,0);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_PRAWY,0);
				  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_PRAWY,0);
			  }
		  }
	  }

	  if(COMM_flaga == 1){

		  COMM_flaga = 0;

		  odebranie = ibus_read(ibus_data);
		  ibus_soft_failsafe(ibus_data, 10);

		  I1_V = ibus_data[0];
		  I2_OMEGA = ibus_data[1];
		  I3_PLCHLDR = ibus_data[2];
		  I4_V_MAX = ibus_data[3];
		  I5_MA = ibus_data[4];
		  I6_ONOFF = ibus_data[5];

		  if (I6_ONOFF > 1800){
			  if (I1_V > I4_V_MAX){ I1_V = I4_V_MAX; }

			  I1_V = to_process_range(2*(I1_V-1500));

			  setpoint_tyl_lewy = I1_V;
			  setpoint_tyl_prawy = I1_V;
		  }

		  else{
			  I1_V = 0;

			  setpoint_tyl_lewy = I1_V;
			  setpoint_tyl_prawy = I1_V;
		  }
		  if (MA == 0){
			  set_motor_dir(&mot_tyl_lewy,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_LEWY,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_LEWY,I1_V);

			  set_motor_dir(&mot_tyl_prawy,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_PRAWY,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_PRAWY,I1_V);
		  }


	  }


	/*Rozmiar = sprintf((char *)Wiadomosc, "czesc:%d\n", test);
	HAL_UART_Transmit(&huart1, (uint8_t*) Wiadomosc,  Rozmiar, 100);*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == IBUS_UART)
		ibus_reset_failsafe();
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim)
{
	if(htim == PID_TIMER){
		PID_flaga = 1;
	}

	if(htim == COMM_TIMER){
		COMM_flaga = 1;
	}
}
int16_t to_process_range(int16_t input){
	int16_t out = (int16_t)(((float)(input * Counter_20kHz_360))/1000.0);
	if (out > Counter_20kHz_360) { out = Counter_20kHz_360; }
	if (out < -Counter_20kHz_360){ out = -Counter_20kHz_360; }
	return out;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

