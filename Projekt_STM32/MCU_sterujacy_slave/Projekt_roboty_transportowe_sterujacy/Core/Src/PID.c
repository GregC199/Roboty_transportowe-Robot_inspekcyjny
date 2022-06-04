/*
 * PID.c
 *
 *  Created on: 6 kwi 2022
 *      Author: Grzegorz Cieslar
 */
#include "PID.h"

void pid_init(PID_t * pid, float p, float i, float d, float dt_ms, int16_t power) {

	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->p_val = 0.0;
	pid->i_val = 0.0;
	pid->d_val = 0.0;

	pid->PID_max_val = FLOAT_MAX;
	pid->PID_min_val = FLOAT_MIN;
	pid->total_max = FLOAT_MAX;
	pid->total_min = FLOAT_MIN;

	pid->e_last = 0.0;
	pid->sum = 0.0;

	pid->dt_ms = dt_ms;

	pid->power = power;

}

int16_t pid_calc(PID_t * pid, int16_t mv, int16_t dv) {

	float p, i, d, e, total;

	//PID process variables
	pid->mv = (float)mv*pid->power;
	pid->dv = (float)dv*pid->power;

	//Error
	e = dv - mv;

	//P
	p = e * pid->p;
	if (p > pid->PID_max_val)
	p = pid->PID_max_val;
	else if (p < pid->PID_min_val)
	p = pid->PID_min_val;
	pid->p_val = p;

	//I
	i = pid->sum;
	i += pid->dt_ms * e / pid->i;
	if (i > pid->PID_max_val)
	i = pid->PID_max_val;
	else if (i < pid->PID_min_val)
	i = pid->PID_min_val;
	pid->sum = i;

	//D
	d = ((e - pid->e_last) * pid->d)/pid->dt_ms;
	if (d > pid->PID_max_val)
	d = pid->PID_max_val;
	else if (d < pid->PID_min_val)
	d = pid->PID_min_val;
	pid->d_val = d;

	//Total
	total = (p + i + d)/(float)pid->power;
	if (total > pid->total_max)
	total = pid->total_max;
	else if (total < pid->total_min)
	total = pid->total_min;

	//Write computed variables
	pid->control = (int16_t)total;
	pid->e_last = e;

	return pid->control;
}

void pid_scaling(PID_t* pid, float  pid_max, float pid_min, float total_max, float total_min)
{
	pid->PID_max_val = pid_max*pid->power;
	pid->PID_min_val = pid_min*pid->power;
	pid->total_max = total_max;
	pid->total_min = total_min;
}
