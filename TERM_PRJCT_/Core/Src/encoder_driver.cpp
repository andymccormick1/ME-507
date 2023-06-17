/*
 * encoder_driver.cpp
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#include "motor_driver.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "encoder_driver.h"
#include "encoder_driver.h"

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


