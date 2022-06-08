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

	m->speedlist[m->iterator] = (int16_t) ( ((float) (m->pulse_count * timer_hz * SEC_IN_MIN)) / (m->resolution*MOT_RPM_TO_1000PROMILE) );

	m->iterator++;

	if (m->iterator >= SMOOTHING) { m->iterator = 0; }

	m->speed = 0;

	for(int i=0; i<SMOOTHING;i++)
	{
		m->speed += m->speedlist[i];
	}

	if(m->DIRECTION == DIR_CCW)
	{
		m->speed = -m->speed;
	}

	return m->speed;
}
void motor_init(motor *m, GPIO_TypeDef * DIR_GPIO_IN,uint16_t DIR_PIN_IN)
{
	m->DIR_PIN = DIR_PIN_IN;

	m->DIR_GPIO = DIR_GPIO_IN;

	m->resolution = (ENCODER_RESOLUTION * TIMER_COUNTS * GEAR_RATIO);

	m->pulse_count = 0;
	m->speed = 0;
	m->iterator = 0;
	for(int i = 0; i<10;i++)
	{
		m->speedlist[i] = 0;
	}
}
void set_motor_dir(motor *m, int16_t process)
{
	if(process > 0)
	{
		HAL_GPIO_WritePin(m->DIR_GPIO, m->DIR_PIN, 0);
		m->DIRECTION = DIR_CW;
	}
	else if(process < 0)
	{
		HAL_GPIO_WritePin(m->DIR_GPIO, m->DIR_PIN, 1);
		m->DIRECTION = DIR_CCW;
	}
}
