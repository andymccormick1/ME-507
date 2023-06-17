/*
 * controller_driver.cpp
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#include "motor_driver.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "controller_driver.h"

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

