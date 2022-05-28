/*
 * PID.h
 *
 *  Created on: 6 kwi 2022
 *      Author: Grzegorz Cieslar
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include <stdint.h>
#define FLOAT_MAX 999999.999
#define FLOAT_MIN -999999.999

typedef struct {

	float p;
	float i;
	float d;
	float p_val;
	float i_val;
	float d_val;

	float PID_max_val;
	float PID_min_val;
	float total_max;
	float total_min;

	float dv;
	float mv;

	float e_last;
	float sum;

	int16_t control;
	int16_t power;
	float dt_ms;

} PID_t;
void pid_init(PID_t* pid, float p, float i, float d, float dt_ms, int16_t power);
int16_t pid_calc(PID_t * pid, int16_t mv, int16_t dv);
void pid_scaling(PID_t* pid, float pid_max, float pid_min, float total_max, float total_min);
#endif /* INC_PID_H_ */
