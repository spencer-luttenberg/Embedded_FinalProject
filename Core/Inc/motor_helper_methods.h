/*
 * motor_helper_methods.h
 *
 *  Created on: Dec 8, 2023
 *      Author: kelly
 */

#ifndef INC_MOTOR_HELPER_METHODS_H_
#define INC_MOTOR_HELPER_METHODS_H_

#include "global_fish_defines.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;


float current_duty_cycle = 0.0;

void set_motor_pwm(uint8_t motor_ID, float duty_cycle){
	switch(motor_ID){
	case MOUTH_MOTOR:
	{
		TIM1->CCR4 = (65535-1)*(duty_cycle/100);
		break;
	}
	case BODY_MOTOR:
	{
		TIM3->CCR1 = (65535-1)*(duty_cycle/100);
		break;
	}
	case TAIL_MOTOR:
	{
		TIM4->CCR2 = (65535-1)*(duty_cycle/100);
		break;
	}
	}
};

void set_fish_motor_states(uint8_t state){
	//Bit format = 0b00000MBT
	switch(state){
	case 0x00:
	{
		//M:0 B:0 T:0
		set_motor_pwm(MOUTH_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		break;
	}
	case 0x01:
	{
		//M:0 B:0 T:1
		set_motor_pwm(MOUTH_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_ON_DUTY_CYCLE);
		break;
	}
	case 0x02:
	{
		//M:0 B:1 T:0
		set_motor_pwm(MOUTH_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_BODY_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		break;
	}
	case 0x03:
	{
		//M:0 B:1 T:1
		set_motor_pwm(MOUTH_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_BODY_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_ON_DUTY_CYCLE);
		break;
	}
	case 0x04:
	{
		//M:1 B:0 T:0
		set_motor_pwm(MOUTH_MOTOR, MOTOR_ON_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		break;
	}
	case 0x05:
	{
		//M:1 B:0 T:1
		set_motor_pwm(MOUTH_MOTOR, MOTOR_ON_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_ON_DUTY_CYCLE);
		break;
	}
	case 0x06:
	{
		//M:1 B:1 T:0
		set_motor_pwm(MOUTH_MOTOR, MOTOR_ON_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_BODY_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_OFF_DUTY_CYCLE);
		break;
	}
	case 0x07:
	{
		//M:1 B:1 T:1
		set_motor_pwm(MOUTH_MOTOR, MOTOR_ON_DUTY_CYCLE);
		set_motor_pwm(BODY_MOTOR, MOTOR_BODY_DUTY_CYCLE);
		set_motor_pwm(TAIL_MOTOR, MOTOR_ON_DUTY_CYCLE);
		break;
	}
	}
};
void reset_fish_motors(){
	set_motor_pwm(MOUTH_MOTOR, MOTOR_OFF_DUTY_CYCLE);
	set_motor_pwm(BODY_MOTOR, MOTOR_OFF_DUTY_CYCLE);
	set_motor_pwm(TAIL_MOTOR, MOTOR_OFF_DUTY_CYCLE);
}





#endif /* INC_MOTOR_HELPER_METHODS_H_ */
