/*
 * motor_driver.c
 *
 *  Created on: May 1, 2023
 *      Author: caleb
 */

#include "motor_driver.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"


// Implementation of default constructor for motor driver class
// the "motor_driver::" prefix indicates that this method belongs to class motor_driver
motor_driver::motor_driver(void)
{

}

// Implementation of initializing constructor
motor_driver::motor_driver(TIM_HandleTypeDef* _htim,
		uint32_t _ch_a,
		uint32_t _ch_b)
    		:htim(_htim),
			 ch_a(_ch_a),
			 ch_b(_ch_b)
{
	htim = _htim;
	ch_a = _ch_a;
	ch_b = _ch_b;
	Duty_Cycle = 0;
	HAL_TIM_PWM_Start(htim,ch_a);
	HAL_TIM_PWM_Start(htim,ch_b);
}


// Implementation of Set PWM Method
void motor_driver::Set_PWM(int16_t	_Duty_Cycle)
{

/**
	  * @brief  Sets the PWM for specified duty cycle for a specified motor driver
	  * @param  Duty_Cycle: int32_t of the desired duty cycle
	  */

	// Saturate the input between -1000 and 1000


	if (_Duty_Cycle < -5000) {
		_Duty_Cycle = -5000;
	} else if (_Duty_Cycle > 5000){
		_Duty_Cycle = 5000;
	}


	Duty_Cycle = _Duty_Cycle;

	int16_t PULSE = abs(Duty_Cycle*4899/10000);	// convert the duty cycle in percentage to the pulse width of PWM signal

	// Set the output PWM channels per the sign of input duty cycle
	if (Duty_Cycle < 0){
		__HAL_TIM_SET_COMPARE(htim,ch_a,4899);
		__HAL_TIM_SET_COMPARE(htim,ch_b,(4899-(PULSE)));
	}
	else {
		__HAL_TIM_SET_COMPARE(htim,ch_a,(4899-PULSE));
		__HAL_TIM_SET_COMPARE(htim,ch_b,4899);
	}
}

// Implementation of method to stop the motor by disabling the PWM
void motor_driver::Stop_Motor(void)
{
	HAL_TIM_PWM_Stop(htim,ch_a);
	HAL_TIM_PWM_Stop(htim,ch_b);
}








