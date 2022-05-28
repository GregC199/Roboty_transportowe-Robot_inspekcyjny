/*
 * MOT.c
 *
 *  Created on: May 27, 2022
 *      Author: ciesl
 */
#include "MOT.h"

void motor_update_count(motor *m, TIM_HandleTypeDef *timer)
{
	m->pulse_count = (int16_t)__HAL_TIM_GET_COUNTER(timer);
	__HAL_TIM_SET_COUNTER(timer, 0);
}
int16_t motor_calculate_speed(motor *m, TIM_HandleTypeDef *timer, int16_t timer_hz)
{
	motor_update_count(m,timer);
	m->speed = (int16_t)(((float)(m->pulse_count * timer_hz * SEC_IN_MIN)) / (m->resolution*MOT_RPM_TO_1000PROMILE));

	if(m->DIRECTION == DIR_CCW)
	{
		m->speed = -m->speed;
	}

	return m->speed;
}
void motor_init(motor *m, GPIO_TypeDef * DIR1_GPIO, GPIO_TypeDef * DIR2_GPIO,uint16_t DIR1_PIN, uint16_t DIR2_PIN)
{
	m->DIR_1_PIN = DIR1_PIN;
	m->DIR_2_PIN = DIR2_PIN;

	m->DIR_1_GPIO = DIR1_GPIO;
	m->DIR_2_GPIO = DIR2_GPIO;

	m->resolution = (ENCODER_RESOLUTION * TIMER_COUNTS * GEAR_RATIO);

	m->pulse_count = 0;
	m->speed = 0;
}
void set_motor_dir(motor *m, int16_t process)
{
	if (process == 0){
		HAL_GPIO_WritePin(m->DIR_1_GPIO, m->DIR_1_PIN, 0);
		HAL_GPIO_WritePin(m->DIR_2_GPIO, m->DIR_2_PIN, 0);
		m->DIRECTION = DIR_STOP;
	}
	else if(process > 0)
	{
		HAL_GPIO_WritePin(m->DIR_1_GPIO, m->DIR_1_PIN, 1);
		HAL_GPIO_WritePin(m->DIR_2_GPIO, m->DIR_2_PIN, 0);
		m->DIRECTION = DIR_CW;
	}
	else if(process < 0)
	{
		HAL_GPIO_WritePin(m->DIR_1_GPIO, m->DIR_1_PIN, 0);
		HAL_GPIO_WritePin(m->DIR_2_GPIO, m->DIR_2_PIN, 1);
		m->DIRECTION = DIR_CCW;
	}
}
