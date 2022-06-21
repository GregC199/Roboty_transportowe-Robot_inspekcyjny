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
#include <stdbool.h>
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

#define MCU_UART (&huart1)
#define MCU_SEND_LEN 14

#define ENCODER_LEWY (&htim2)
#define ENCODER_PRAWY (&htim3)
#define PWM_TIMER_SILNIKI (&htim1)
#define IC_TIMER_SERWO (&htim16)
#define COMM_TIMER (&htim6)
#define COMM_HZ 10

#define CW 0
#define CCW 1

//DEFINES INSIDE MOT.h OF MOTOR PROPERTIES

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t MCU_send_data[MCU_SEND_LEN];

bool odebranie;
bool MA;
bool OFF_ON;

int8_t COMM_flaga = 0;

uint16_t ibus_data[IBUS_USER_CHANNELS];

int16_t pomiar_przod_lewy = 0;
int16_t pomiar_przod_prawy = 0;

int16_t pomiar_serwo_kat = 0;

motor mot_przod_lewy;
motor mot_przod_prawy;
int16_t ms = 50;

int16_t I1_V;
int16_t I2_OMEGA;
int16_t I3_PLCHLDR;
int16_t I4_V_MAX;
int16_t I5_MA;
int16_t I6_ONOFF;

uint32_t IC_Captured = 0;
uint32_t T0 = 0;
uint32_t T1 = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim);
int16_t to_process_range(int16_t input);
int16_t to_communication_range(int16_t input, int16_t replacement);
bool check_communication(int16_t input);
int16_t to_DAC(int16_t input, float min, float max);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //INIT komunikacji ze zdalna aparatura
  ibus_init();
  __HAL_UART_ENABLE_IT(MCU_UART, UART_IT_RXNE);  // Interrupt Enable
  __HAL_UART_ENABLE_IT(MCU_UART, UART_IT_TC);

  //komunikacja timer
  HAL_TIM_Base_Start_IT(COMM_TIMER);
  HAL_TIM_IC_Start_IT(IC_TIMER_SERWO, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(ENCODER_LEWY, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENCODER_PRAWY, TIM_CHANNEL_ALL);


  //Motor inicjalizacja
  motor_init(&mot_przod_lewy);
  motor_init(&mot_przod_prawy);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //ODBIOR Z KOMUNIKACJI I OBROBKA DANYCH
	  if(COMM_flaga == 1){

		  COMM_flaga = 0;

		  //LEWE
		  pomiar_przod_lewy = to_process_range(motor_calculate_speed(&mot_przod_lewy, ENCODER_LEWY, COMM_HZ));

		  //PRAWE
		  pomiar_przod_prawy = to_process_range(motor_calculate_speed(&mot_przod_prawy, ENCODER_PRAWY, COMM_HZ));

		  //SERWO
		  pomiar_serwo_kat = to_process_range((int16_t)((((float)(T0))/ ((float)(T0+T1)))*1000.0));

		  odebranie = ibus_read(ibus_data);
		  ibus_soft_failsafe(ibus_data, 10);

		  I1_V = to_communication_range(ibus_data[1], 1500);
		  I2_OMEGA = to_communication_range(ibus_data[0],1500);
		  I3_PLCHLDR = to_communication_range(ibus_data[3],1500);
		  I4_V_MAX = to_communication_range(ibus_data[4],2000);
		  I5_MA = to_communication_range(ibus_data[2],0);
		  if (I5_MA == 0){ibus_data[5] = 1000;} 				//JEŚLI NIE MA TRYBU W ODPOWIEDNIM RANGE TO STOP
		  I6_ONOFF = to_communication_range(ibus_data[5],1000);

		  //SPROWADZENIE WARTOSCI DO BOOL
		  OFF_ON = check_communication(I6_ONOFF);
		  MA = check_communication(I5_MA);

		  I1_V = to_process_range(2*(I1_V-1500));
		  I2_OMEGA = to_process_range(2*(I2_OMEGA-1500));
		  I4_V_MAX = to_process_range((I4_V_MAX-1000));

		  //JEŚLI WY�?ĄCZONY TO TRYB MANUALNY NADPISUJACY ZEROWE STEROWANIA
		  if (OFF_ON == 0){ MA = 0;}


		  //W�?ĄCZONY
		  if (OFF_ON == 1){
			  if (I1_V > I4_V_MAX){ I1_V = I4_V_MAX; }
			  if (I1_V < -I4_V_MAX){ I1_V = -I4_V_MAX; }
		  }

		  //WY�?ĄCZONY - WPISANIE ZERA
		  else{
			  I1_V = 0;
		  }
		  if(OFF_ON == 1){MCU_send_data[0] = 1;}
		  else{MCU_send_data[0] = 0;}
		  if(MA == 1){MCU_send_data[1] = 1;}
		  else{MCU_send_data[1] = 0;}
		  MCU_send_data[2] = (uint8_t)pomiar_przod_lewy;
		  MCU_send_data[3] = (uint8_t)(pomiar_przod_lewy >> 8);
		  MCU_send_data[4] = (uint8_t)pomiar_przod_prawy;
		  MCU_send_data[5] = (uint8_t)(pomiar_przod_prawy >> 8);
		  MCU_send_data[6] = (uint8_t)pomiar_serwo_kat;
		  MCU_send_data[7] = (uint8_t)(pomiar_serwo_kat >> 8);
		  MCU_send_data[8] = (uint8_t)I1_V;
		  MCU_send_data[9] = (uint8_t)(I1_V >> 8);
		  MCU_send_data[10] = (uint8_t)I2_OMEGA;
		  MCU_send_data[11] = (uint8_t)(I2_OMEGA >> 8);
		  MCU_send_data[12] = (uint8_t)I4_V_MAX;
		  MCU_send_data[13] = (uint8_t)(I4_V_MAX >> 8);
		  HAL_UART_Transmit_IT(MCU_UART, MCU_send_data, sizeof(MCU_send_data));


	  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == IBUS_UART) { ibus_reset_failsafe();}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim)
{
	if(htim == COMM_TIMER){
		COMM_flaga = 1;
	}
}

int16_t to_process_range(int16_t input)
{
	int16_t out = (int16_t)(((float)(input * Counter_20kHz_360))/1000.0);

	if (out > Counter_20kHz_360) { out = Counter_20kHz_360; }
	else if (out < -Counter_20kHz_360) { out = -Counter_20kHz_360; }

	return out;
}

int16_t to_communication_range(int16_t input, int16_t replacement)
{
	int16_t out;

	if (input < 1000) { out = replacement;}
	else if (input > 2000) { out = replacement;}
	else { out = input;}

	return out;
}

bool check_communication(int16_t input)
{
	bool out;

	if (input < 1050) { out = 0;}
	else if (input > 1950) { out = 1;}
	else { out = 0;}

	return out;
}

int16_t to_DAC(int16_t input, float min, float max)
{
	int16_t out;

	out = (int16_t) ( ((float)(input - 1000) /1000.0) * ( max - min ) + min );

	return out;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == IC_TIMER_SERWO)
	{
		if (IC_Captured == 0)
		{
			T0 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			IC_Captured = 1;
			__HAL_TIM_SET_COUNTER(htim, 0);
		}
		else
		{
			T1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			__HAL_TIM_SET_COUNTER(htim, 0);
			IC_Captured = 0;
		}
	}
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

