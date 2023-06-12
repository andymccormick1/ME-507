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


	if (_Duty_Cycle < -10000) {
		_Duty_Cycle = -10000;
	} else if (_Duty_Cycle > 10000){
		_Duty_Cycle = 10000;
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



// Implementation of default constructor for encoder reader class
encoder_reader::encoder_reader(void)
{

}

// Implementation of initializing constructor
encoder_reader::encoder_reader(TIM_HandleTypeDef* _htim)
    		:htim(_htim)

{
	htim = _htim;
	HAL_TIM_Encoder_Start(htim,TIM_CHANNEL_ALL);

	// Initialize delta and count to zero
	COUNT = 0;
	DELTA = 0;
	prev_value 	= 	__HAL_TIM_GET_COUNTER(htim); // Initialize the previous value to current timer count
}

// Implementation of method to zero the current encoder position
void encoder_reader::zero_count(void)
{
	COUNT = 0;
	prev_value 	= 	__HAL_TIM_GET_COUNTER(htim);
}


// Implementation of method to get the current encoder position and store in the object
int32_t encoder_reader::get_count(void)
{
	DELTA 		= 	__HAL_TIM_GET_COUNTER(htim) - prev_value;	// calculate the change in encoder position
	prev_value 	= 	__HAL_TIM_GET_COUNTER(htim);			// store current value for next encoder read callback


	if (DELTA > 32768){		// overflow condition
		DELTA -= 65536;
	}

	else if (DELTA < -32768){
		DELTA += 65536;		// under flow condition
	}

	COUNT += DELTA;			// increment counter delta

	return COUNT;
}

// Implementation of the feedback controller class
feedback_controller::feedback_controller(void)
{
	// Initialize all gains, error and setpoints to zero
	KP = 0;
	KI = 0;
	DUTY_CYCLE = 0;
	SET_POINT = 0;
	CURR_CNT = 0;
	INT_ERR = 0;
}

// Implementation of method to set the setpoint of feedback controller
void feedback_controller::set_setpoint(int32_t _SET_POINT)
{
	SET_POINT = _SET_POINT;
}

// Implementation of method to set the porportional gain
void feedback_controller::set_KP(int16_t _KP)
{
	KP = _KP;
}


// Implementation of method to set the integral gain
void feedback_controller::set_KI(int16_t _KI)
{
	KI = _KI;
	INT_ERR = 0;	// zero the integral error when setting KI
}

// Implementation of the method to run porportional - integral control based on curent encoder position
int16_t feedback_controller::run(int32_t _CURR_CNT)
{
	CURR_CNT = _CURR_CNT;
	INT_ERR += (SET_POINT - CURR_CNT);			// add the current error to the integrated error

	DUTY_CYCLE = KP*(SET_POINT - CURR_CNT) + KI*INT_ERR/100;	// Determine duty cycle from KP and KI
	return DUTY_CYCLE;
}
