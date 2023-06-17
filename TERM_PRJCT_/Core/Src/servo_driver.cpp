/*
 * servo_driver.cpp
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#include "motor_driver.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "servo_driver.h"


// Implementation of default constructor for motor driver class
// the "motor_driver::" prefix indicates that this method belongs to class motor_driver
servo_driver::servo_driver(void)
{

}

// Implementation of initializing constructor
servo_driver::servo_driver(TIM_HandleTypeDef* _htim,
		uint32_t _channel)
    		:htim(_htim),
			 channel(_channel)
{
	htim = _htim;
	channel = _channel;
	ANGLE = 0;
	HAL_TIM_PWM_Start(htim,channel);
}


// Implementation of Set PWM Method
void servo_driver::Set_Position(int16_t	_ANGLE)
{

/**
	  * @brief  Sets the PWM for specified duty cycle for a specified motor driver
	  * @param  Duty_Cycle: int32_t of the desired duty cycle
	  */

	// Saturate the input between -90 and 90
	ANGLE = _ANGLE;

	if (ANGLE < -90) {
		ANGLE = -90;
	} else if (ANGLE > 90){
		ANGLE = 90;
	}

	int16_t PULSE = 0;		// intialize pulse signal
	if (ANGLE<0){
		PULSE = 1500 + ANGLE*500/90;
	}
	else{
		PULSE = 1500 + ANGLE*500/90;
	}

	__HAL_TIM_SET_COMPARE(htim,channel,PULSE);

	}

