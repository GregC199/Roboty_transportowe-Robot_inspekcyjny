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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//https://github.com/mokhwasomssi/stm32_hal_ibus
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
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
#define PC_UART (&huart2)
#define PC_RECV_LEN 5
#define PC_SEND_LEN 21

#define MCU_UART (&huart1)
#define MCU_LEN 14

#define ENCODER_LEWY (&htim2)
#define ENCODER_PRAWY (&htim3)
#define PWM_TIMER_SILNIKI (&htim1)
#define PWM_TIMER_SERWO (&htim17)
#define PID_TIMER (&htim6)
#define COMM_TIMER (&htim7)
#define COMM_HZ 10
#define PID_HZ 10

#define CW 0
#define CCW 1

#define PID_POWER 1.0

#define CH_TYL_LEWY                        0x00000000U                          /*!< Capture/compare channel 1 identifier      */
#define CH_TYL_PRAWY                       0x00000004U                          /*!< Capture/compare channel 2 identifier      */
#define CH_PRZOD_LEWY                      0x00000008U                          /*!< Capture/compare channel 3 identifier      */
#define CH_PRZOD_PRAWY 					   0x0000000CU

#define CH_SERWO                        0x00000000U

#define KP 1.0
#define TI 1.0
#define TD 0.0

#define L1 43.0
#define L2 10.0
#define SERWO_SCALE_MAX 25
#define LEWE_KOLO 0
#define PRAWE_KOLO 1
#define L_ADD 50
//DEFINES INSIDE MOT.h OF MOTOR PROPERTIES

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int8_t Wiadomosc[200];
//int16_t Rozmiar;
//int16_t test = 10;

bool MA = 0;
bool OFF_ON = 1;

bool krancowka = 0;
bool STOP_AW = 0;

int8_t PID_flaga = 0;
int8_t COMM_flaga = 0;
int8_t UART_flaga = 0;

uint8_t mcu_data[MCU_LEN];
uint8_t pc_recv_data[PC_RECV_LEN];
uint8_t pc_send_data[PC_SEND_LEN];

uint16_t recv_pc  = 0;
uint16_t recv_mcu  = 0;

float r1;
float r2;

//TYL
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

//Przod
int16_t sterowanie_przod_lewy = 0;
int16_t sterowanie_przod_prawy = 0;

int16_t pomiar_przod_lewy = 0;
int16_t pomiar_przod_prawy = 0;

int16_t setpoint_przod_lewy = 0;
int16_t setpoint_przod_prawy = 0;

PID_t Pid_przod_lewy;
PID_t Pid_przod_prawy;

motor mot_przod_lewy;
motor mot_przod_prawy;

float ms = 0.1;

int16_t I1_V = 0;
int16_t I2_OMEGA = 0;
int16_t I4_Vmax = 0;

int16_t pomiar_serwo_kat = 0;
int16_t sterowanie_serwo_kat = 0;

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim);
int16_t to_process_range(int16_t input);
int16_t przelicz_kat(int16_t setpoint, bool L_P);
//int16_t to_communication_range(int16_t input, int16_t replacement);
//bool check_communication(int16_t input);
//int16_t to_DAC(int16_t input, float min, float max);

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //INIT komunikacji ze zdalna aparatura
  //ibus_init();

  //timery dla PID i komunikacji z PC
  HAL_TIM_Base_Start_IT(PID_TIMER);
  HAL_TIM_Base_Start_IT(COMM_TIMER);
  HAL_TIM_Encoder_Start(ENCODER_LEWY, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENCODER_PRAWY, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(PWM_TIMER_SILNIKI, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(PWM_TIMER_SILNIKI, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(PWM_TIMER_SILNIKI, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(PWM_TIMER_SILNIKI, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(PWM_TIMER_SERWO, TIM_CHANNEL_1);
  __HAL_UART_ENABLE_IT(MCU_UART, UART_IT_RXNE);  // Interrupt Enable
  __HAL_UART_ENABLE_IT(MCU_UART, UART_IT_TC);

  //Pid inicjalizacja
  ms = 1.0/(PID_HZ);
  pid_init(&Pid_tyl_lewy, KP, TI, TD, ms,PID_POWER);
  pid_scaling(&Pid_tyl_lewy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);
  pid_init(&Pid_tyl_prawy, KP, TI, TD, ms,PID_POWER);
  pid_scaling(&Pid_tyl_prawy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);
  pid_init(&Pid_przod_lewy, KP, TI, TD, ms,PID_POWER);
  pid_scaling(&Pid_przod_lewy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);
  pid_init(&Pid_przod_prawy, KP, TI, TD, ms,PID_POWER);
  pid_scaling(&Pid_przod_prawy, Counter_20kHz_360, -Counter_20kHz_360, Counter_20kHz_360, -Counter_20kHz_360);

  //Motor inicjalizacja
  motor_init(&mot_tyl_lewy,DIR_LT_GPIO_Port,DIR_LT_Pin);
  motor_init(&mot_tyl_prawy,DIR_PT_GPIO_Port,DIR_PT_Pin);
  motor_init(&mot_przod_lewy,DIR_LP_GPIO_Port,DIR_LP_Pin);
  motor_init(&mot_przod_prawy,DIR_PP_GPIO_Port,DIR_PP_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(PID_flaga == 1){

		  PID_flaga = 0;

		  //LEWE TYL - PID OBLICZA NAWET W TRYBIE MANUALNYM
		  pomiar_tyl_lewy = to_process_range(motor_calculate_speed(&mot_tyl_lewy, ENCODER_LEWY, PID_HZ));
		  sterowanie_tyl_lewy = pid_calc(&Pid_tyl_lewy, pomiar_tyl_lewy, setpoint_tyl_lewy);

		  //PRZOD
		  sterowanie_przod_lewy = pid_calc(&Pid_przod_lewy, pomiar_przod_lewy, setpoint_przod_lewy);

		  //PRAWE TYL
		  pomiar_tyl_prawy = to_process_range(motor_calculate_speed(&mot_tyl_prawy, ENCODER_PRAWY, PID_HZ));
		  sterowanie_tyl_prawy = pid_calc(&Pid_tyl_prawy, pomiar_tyl_prawy, setpoint_tyl_prawy);

		  //PRZOD
		  sterowanie_przod_prawy = pid_calc(&Pid_przod_prawy, pomiar_przod_prawy, setpoint_przod_prawy);

		  //STEROWANIE DLA TRYBU AUTOMATYCZNEGO
		  //if(MA == 1){  TU I NA KONCU ZAKOMENTOWANE
		  if (I1_V != 0){

			  if(sterowanie_tyl_lewy > I4_Vmax){
				  sterowanie_tyl_lewy = I4_Vmax;

			  }
			  else if(sterowanie_tyl_lewy < -I4_Vmax){
				  sterowanie_tyl_lewy = -I4_Vmax;
			  }
			  if(sterowanie_tyl_prawy > I4_Vmax){
				  sterowanie_tyl_prawy = I4_Vmax;

			  }
			  else if(sterowanie_tyl_prawy < -I4_Vmax){
				  sterowanie_tyl_prawy = -I4_Vmax;
			  }

			  if(sterowanie_przod_lewy > I4_Vmax){
				  sterowanie_przod_lewy = I4_Vmax;

			  }
			  else if(sterowanie_przod_lewy < -I4_Vmax){
				  sterowanie_przod_lewy = -I4_Vmax;
			  }
			  if(sterowanie_przod_prawy > I4_Vmax){
				  sterowanie_przod_prawy = I4_Vmax;

			  }
			  else if(sterowanie_przod_prawy < -I4_Vmax){
				  sterowanie_przod_prawy = -I4_Vmax;
			  }



			  set_motor_dir(&mot_tyl_lewy,sterowanie_tyl_lewy);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_TYL_LEWY,abs(sterowanie_tyl_lewy));
			  set_motor_dir(&mot_przod_lewy,sterowanie_przod_lewy);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_PRZOD_LEWY,abs(sterowanie_przod_lewy));

			  set_motor_dir(&mot_tyl_prawy,sterowanie_tyl_prawy);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_TYL_PRAWY,abs(sterowanie_tyl_prawy));
			  set_motor_dir(&mot_przod_prawy,sterowanie_przod_prawy);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_PRZOD_PRAWY,abs(sterowanie_przod_prawy));
		  }
		  else{
			  set_motor_dir(&mot_tyl_lewy,0);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_TYL_LEWY,0);
			  set_motor_dir(&mot_przod_lewy,0);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_PRZOD_LEWY,0);

			  set_motor_dir(&mot_tyl_prawy,0);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_TYL_PRAWY,0);
			  set_motor_dir(&mot_przod_prawy,0);
			  __HAL_TIM_SetCompare(PWM_TIMER_SILNIKI, CH_PRZOD_PRAWY,0);
		  }
		  //}
	  }

	  /*
	  //STEROWANIE KOLAMI
	  if(PID_flaga == 1){
		  PID_flaga = 0;

		  //LEWE - PID OBLICZA NAWET W TRYBIE MANUALNYM
		  pomiar_tyl_lewy = to_process_range(motor_calculate_speed(&mot_tyl_lewy, ENCODER_LEWY, PID_HZ));
		  sterowanie_tyl_lewy = pid_calc(&Pid_tyl_lewy, pomiar_tyl_lewy, setpoint_tyl_lewy);

		  //STEROWANIE DLA TRYBU AUTOMATYCZNEGO
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

		  //PRAWE - PID OBLICZA NAWET W TRYBIE MANUALNYM
		  pomiar_tyl_prawy = to_process_range(motor_calculate_speed(&mot_tyl_prawy, ENCODER_PRAWY, PID_HZ));
		  sterowanie_tyl_prawy = pid_calc(&Pid_tyl_prawy, pomiar_tyl_prawy, setpoint_tyl_prawy);

		  //STEROWANIE DLA TRYBU AUTOMATYCZNEGO
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
	  } */

	  if(COMM_flaga == 1){
		  COMM_flaga = 0;

		  //Odbior danych
		  if(UART_flaga == 1)
		  {
			  UART_flaga = 0;
			  //recv_mcu = HAL_UART_Receive_IT(MCU_UART,mcu_data,sizeof(mcu_data));
		  }
		  recv_mcu = HAL_UART_Receive_IT(MCU_UART,mcu_data,sizeof(mcu_data));
		  //recv_mcu = HAL_UART_Receive(MCU_UART,mcu_data,sizeof(mcu_data),100);
		  recv_pc = HAL_UART_Receive(PC_UART,pc_recv_data,PC_RECV_LEN,200);

		  //Przetwarzanie warunkow dla OFF ON oraz MA
		  if(mcu_data[0] == 1){OFF_ON = 1;}
		  else{OFF_ON = 0;}

		  if(mcu_data[1] == 1){MA = 1;}
		  else if(mcu_data[1] == 0){MA = 0;}
		  else
		  {
			  MA = 0;
			  OFF_ON = 0;
		  }

		  //Odczyt pomiarow z enkoderow na masterze
		  pomiar_przod_lewy = (int16_t)((mcu_data[3] << 8) | mcu_data[2]);
		  if(mot_przod_lewy.DIRECTION == DIR_CCW)
		  {
			  pomiar_przod_lewy = -pomiar_przod_lewy;
		  }
		  pomiar_przod_prawy = (int16_t)((mcu_data[5] << 8) | mcu_data[4]);
		  if(mot_przod_prawy.DIRECTION == DIR_CCW)
		  {
			  pomiar_przod_prawy = -pomiar_przod_prawy;
		  }

		  //Zwrotka z serwo i maksymalna predkosc
		  pomiar_serwo_kat = (int16_t)((mcu_data[7] << 8) | mcu_data[6]);
		  I4_Vmax = (int16_t)((mcu_data[13] << 8) | mcu_data[12]);

		  //Jeśli MA == 0 to odczyt z PILOTA z mastera
		  if(MA == 0){
			  I1_V = (int16_t)((mcu_data[9] << 8) | mcu_data[8]);
			  I2_OMEGA = (int16_t)((mcu_data[11] << 8) | mcu_data[10]);
		  }

		  //Jeśli MA == 1 to odczyt z PC
		  if(MA == 1){

			  I1_V = (int16_t)((pc_recv_data[2] << 8) | pc_recv_data[1]);
			  I2_OMEGA = (int16_t)((pc_recv_data[4] << 8) | pc_recv_data[3]);

			  if(pc_recv_data[0] == 1){OFF_ON = 1;}
			  else{OFF_ON = 0;}
		  }

		  if(STOP_AW == 1){
			  OFF_ON = 0;
			  MA = 0;
		  }

		  if(OFF_ON == 0){
			  I1_V = 0;
			  I2_OMEGA = 0;
			  sterowanie_serwo_kat = 0;

			  HAL_GPIO_WritePin(WYLACZNIK_GPIO_Port, WYLACZNIK_Pin, 1);
		  }
		  else if(OFF_ON == 1){
			  HAL_GPIO_WritePin(WYLACZNIK_GPIO_Port, WYLACZNIK_Pin, 0);
		  }

		  //Sterowanie serwonapędem oraz sprawdzenie ograniczeń
		  if (OFF_ON == 1){
			  if (krancowka == 1){
				  if (sterowanie_serwo_kat > 0){
					  I2_OMEGA = sterowanie_serwo_kat - 2;

				  }
				  else{
					  I2_OMEGA = sterowanie_serwo_kat + 2;
				  }
			  }
			  else{sterowanie_serwo_kat = I2_OMEGA;}

			  //Sprawdzenie ograniczeń predkosci

			  if(I1_V > I4_Vmax){
				  I1_V = I4_Vmax;

			  }
			  else if(I1_V < -I4_Vmax){
				  I1_V = -I4_Vmax;
			  }
		  }

		  if(I2_OMEGA >= 0){HAL_GPIO_WritePin(DIR_SERWO_GPIO_Port, DIR_SERWO_Pin, 0);}
		  else{HAL_GPIO_WritePin(DIR_SERWO_GPIO_Port, DIR_SERWO_Pin, 1);}
		  __HAL_TIM_SetCompare(PWM_TIMER_SERWO, CH_SERWO,abs(I2_OMEGA));

		  setpoint_przod_lewy = przelicz_kat(I1_V,0);
		  setpoint_przod_prawy = przelicz_kat(I1_V,1);
		  setpoint_tyl_lewy = I1_V;
		  setpoint_tyl_prawy = I1_V;



		  //Transmisja danych UARTEM do PC
		  if(OFF_ON == 1){pc_send_data[0] = 1;}
		  else{pc_send_data[0] = 0;}
		  pc_send_data[1] = (uint8_t)pomiar_tyl_lewy;
		  pc_send_data[2] = (uint8_t)(pomiar_tyl_lewy >> 8);
		  pc_send_data[3] = (uint8_t)pomiar_tyl_prawy;
		  pc_send_data[4] = (uint8_t)(pomiar_tyl_prawy >> 8);
		  pc_send_data[5] = (uint8_t)pomiar_przod_lewy;
		  pc_send_data[6] = (uint8_t)(pomiar_przod_lewy >> 8);
		  pc_send_data[7] = (uint8_t)pomiar_przod_prawy;
		  pc_send_data[8] = (uint8_t)(pomiar_przod_prawy >> 8);
		  pc_send_data[9] = (uint8_t)pomiar_serwo_kat;
		  pc_send_data[10] = (uint8_t)(pomiar_serwo_kat >> 8);
		  pc_send_data[11] = (uint8_t)sterowanie_tyl_lewy;
		  pc_send_data[12] = (uint8_t)(sterowanie_tyl_lewy >> 8);
		  pc_send_data[13] = (uint8_t)sterowanie_tyl_prawy;
		  pc_send_data[14] = (uint8_t)(sterowanie_tyl_prawy >> 8);
		  pc_send_data[15] = (uint8_t)sterowanie_przod_lewy;
		  pc_send_data[16] = (uint8_t)(sterowanie_przod_lewy >> 8);
		  pc_send_data[17] = (uint8_t)sterowanie_przod_prawy;
		  pc_send_data[18] = (uint8_t)(sterowanie_przod_prawy >> 8);
		  pc_send_data[19] = (uint8_t)sterowanie_serwo_kat;
		  pc_send_data[20] = (uint8_t)(sterowanie_serwo_kat >> 8);
		  if(MA == 1){pc_send_data[21] = 1;}
		  else if(MA == 0){
			  pc_send_data[21] = 0;
		  }
		  else{pc_send_data[21] = 3;}
		  HAL_UART_Transmit(PC_UART, pc_send_data, PC_SEND_LEN, 200);

	  }
	  /*
	  //ODBIOR Z KOMUNIKACJI I OBROBKA DANYCH
	  if(COMM_flaga == 1){

		  COMM_flaga = 0;

		  odebranie = ibus_read(ibus_data);
		  ibus_soft_failsafe(ibus_data, 10);

		  I1_V = to_communication_range(ibus_data[0], 1500);
		  I2_OMEGA = to_communication_range(ibus_data[1],1500);
		  I3_PLCHLDR = to_communication_range(ibus_data[2],1500);
		  I4_V_MAX = to_communication_range(ibus_data[3],2000);
		  I5_MA = to_communication_range(ibus_data[4],0);
		  if (I5_MA == 0){ibus_data[5] = 1000;} 				//JEŚLI NIE MA TRYBU W ODPOWIEDNIM RANGE TO STOP
		  I6_ONOFF = to_communication_range(ibus_data[5],1000);

		  //SPROWADZENIE WARTOSCI DO BOOL
		  OFF_ON = check_communication(I6_ONOFF);
		  MA = check_communication(I5_MA);

		  //JEŚLI WY�?ĄCZONY TO TRYB MANUALNY NADPISUJACY ZEROWE STEROWANIA
		  if (OFF_ON == 0){ MA = 0;}


		  //W�?ĄCZONY
		  if (OFF_ON == 1){
			  if (I1_V > I4_V_MAX){ I1_V = I4_V_MAX; }

			  I1_V = to_process_range(2*(I1_V-1500));

			  setpoint_tyl_lewy = I1_V;
			  setpoint_tyl_prawy = I1_V;
		  }

		  //WY�?ĄCZONY - WPISANIE ZER
		  else{
			  I1_V = 0;

			  setpoint_tyl_lewy = I1_V;
			  setpoint_tyl_prawy = I1_V;
		  }

		  //STEROWANIE DLA TRYBU MANUALNEGO
		  if (MA == 0){
			  set_motor_dir(&mot_tyl_lewy,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_LEWY,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_LEWY,I1_V);

			  set_motor_dir(&mot_tyl_prawy,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_TYL_PRAWY,I1_V);
			  __HAL_TIM_SetCompare(PWM_TIMER, CH_PRZOD_PRAWY,I1_V);
		  }


	  }


	Rozmiar = sprintf((char *)Wiadomosc, "czesc:%d\n", test);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if(huart == IBUS_UART) { ibus_reset_failsafe();}
	/*if(huart == MCU_UART) {
		UART_flaga = 1;
		recv_mcu = HAL_UART_Receive_IT(MCU_UART,mcu_data,sizeof(mcu_data));}
*/
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

int16_t to_process_range(int16_t input)
{
	int16_t out = (int16_t)(((float)(input * Counter_20kHz_360))/1000.0);

	if (out > Counter_20kHz_360) { out = Counter_20kHz_360; }
	else if (out < -Counter_20kHz_360) { out = -Counter_20kHz_360; }

	return out;
}
int16_t przelicz_kat(int16_t setpoint, bool L_P){
	bool dir;
	float stosunek;
	float predkosc;

	int16_t tmp = (setpoint * SERWO_SCALE_MAX) / Counter_20kHz_360;
	if(setpoint >= 0){
		dir = 0;
	}
	else{dir = 1;}

	r1 = (L1*cos(tmp) + L2)/sin(tmp);
	r2 = (L2*cos(tmp) + L1)/sin(tmp);

	predkosc = (float)( (((float)tmp) * (r1 - ( ((float)L_ADD) /2.0) )) / r2);
	stosunek = (r1 + ( ((float)L_ADD) /2.0))/(r1 - ( ((float)L_ADD) /2.0) );

	if( (L_P == 0) && (dir == 0) ){
		return (int16_t)(stosunek*predkosc);
	}
	else if( (L_P == 0) && (dir == 1) ){
			return (int16_t)(predkosc);
	}
	else if( (L_P == 1) && (dir == 1) ){
			return (int16_t)(stosunek*predkosc);
	}
	else if( (L_P == 1) && (dir == 0) ){
			return (int16_t)(predkosc);
	}
	else{return 0;}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == STOP_AW_Pin)
    {
    	if(HAL_GPIO_ReadPin(STOP_AW_GPIO_Port, STOP_AW_Pin) == GPIO_PIN_SET){
    		STOP_AW = 1;
    	}
    	if(HAL_GPIO_ReadPin(STOP_AW_GPIO_Port, STOP_AW_Pin) == GPIO_PIN_RESET){
    	    STOP_AW = 0;
    	}

    }

    if(GPIO_Pin == KRANCOWKA_1_Pin)
    {
    	if((HAL_GPIO_ReadPin(KRANCOWKA_1_GPIO_Port, KRANCOWKA_1_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(KRANCOWKA_2_GPIO_Port, KRANCOWKA_2_Pin) == GPIO_PIN_SET)){
    		krancowka = 1;
    	}
    	if((HAL_GPIO_ReadPin(KRANCOWKA_1_GPIO_Port, KRANCOWKA_1_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(KRANCOWKA_2_GPIO_Port, KRANCOWKA_2_Pin) == GPIO_PIN_RESET)){
    		krancowka = 0;
    	}
    }

    if(GPIO_Pin == KRANCOWKA_2_Pin)
    {
    	if((HAL_GPIO_ReadPin(KRANCOWKA_1_GPIO_Port, KRANCOWKA_1_Pin) == GPIO_PIN_SET) || (HAL_GPIO_ReadPin(KRANCOWKA_2_GPIO_Port, KRANCOWKA_2_Pin) == GPIO_PIN_SET)){
    		krancowka = 1;
    	}
    	if((HAL_GPIO_ReadPin(KRANCOWKA_1_GPIO_Port, KRANCOWKA_1_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(KRANCOWKA_2_GPIO_Port, KRANCOWKA_2_Pin) == GPIO_PIN_RESET)){
    		krancowka = 0;
    	}
    }

}
/*
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
}*/

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

