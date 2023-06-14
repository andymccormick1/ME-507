/*
 * sorting_task.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: andymccormick
 */

#include "movement_task.h"
#include "motor_driver.h"
#include "BNO055.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

sort_task::sort_task(servo_driver servo,
			UART_HandleTypeDef* huart)

				:state(0), runs(0),
				state_list(new state_fcn[3]{(sort_task::state_fcn) &sort_task::state_0,
						(sort_task::state_fcn) &sort_task::state_1,
						(sort_task::state_fcn) &sort_task::state_2),
						num_states(3),
				servo_driver(_servo_driver),
				servo_angle(0),
				huart(_huart),
				start_ticks(0),
				curr_time(0),
				start_count(0)
{
	printf("Initialized?");
}

void movement_task::run(void)
{
	if (state>=0 && state<num_states)
	{
		(this->*state_list[state])();

		runs++;
	}
	else
	{
		printf("Error Invalid State");
		while(1){}
	}
}

void movement_task::state_0(void)
{
	// Initialization state
	// Set servo to zero

	servo_driver.Set_Position(servo_angle);
	state = 1;
}

void movement_task::state_1(void)
{
	// Sort the ball from RGB data

	start_ticks = 1;

}

void movement_task::state_2(void)
{
	if (start_ticks == 1){
		start_count =  HAL_GetTick();
		start_ticks = 0;
	}
	curr_time = HAL_GetTick() - start_count;	// elapsed time in the current state
	servo_driver.Set_Position(servo_angle);		

	if (curr_time > 2000){		// Transition to sort balls after 2 seconds and return to 0 degree position
		servo_angle = 0;
		servo_driver.Set_Position(servo_angle);
		state = 1;
	}

}
