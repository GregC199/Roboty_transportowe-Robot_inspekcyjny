/*
 * MOT.h
 *
 *  Created on: May 27, 2022
 *      Author: ciesl
 */

#ifndef INC_MOT_H_
#define INC_MOT_H_

#define ENCODER_RESOLUTION 1024.0
#define TIMER_COUNTS 4.0
#define GEAR_RATIO 1.78 //9.78
#define SEC_IN_MIN 60.0
#define MOT_RPM_TO_1000PROMILE 0.3 //300 rpm; 3000 rated motor
#define DIR_CW 0
#define DIR_CCW 1
#define DIR_STOP 2
#define SMOOTHING 20

#include <stdint.h>
#include "tim.h"
typedef struct
{
	GPIO_TypeDef * DIR_GPIO;
	uint16_t       DIR_PIN;
	int16_t        DIRECTION;
	int16_t        iterator;
	int16_t        speedlist[SMOOTHING];
	float 		   resolution;		//rozdzielczość silnika
	int16_t 	   pulse_count;		//zliczone impulsy
	int16_t 	   speed;			//obliczona prędkość silnika
}motor;
void motor_update_count(motor *m, TIM_HandleTypeDef *timer);
int16_t motor_calculate_speed(motor *m, TIM_HandleTypeDef *timer, int16_t timer_hz);
void motor_init(motor *m, GPIO_TypeDef * DIR_GPIO_IN,uint16_t DIR_PIN_IN);
void set_motor_dir(motor *m, int16_t process);
#endif /* INC_MOT_H_ */
