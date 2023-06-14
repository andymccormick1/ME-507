/*
 * sorting_task/h
 *
 *  Created on: Jun 13, 2023
 *      Author: caleb erlenborn
 */

#ifndef sorting_task_TASK_H_
#define sorting_task_TASK_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "BNO055.h"
#include "motor_driver.h"

class sort_task
{
private:

	typedef void (sort_task::*state_fcn)(void);
	uint32_t state; // Current state

	uint32_t runs; // Number of times Task Has Ran

	uint32_t num_states; // Number of states

	state_fcn* state_list; // Pointer to a List of States

	void state_0(void); // Initialize state
	void state_1(void); // Wait for color state
	void state_2(void); // Set servo position state

	servo_driver servo;
	UART_HandleTypeDef * huart;


	int16_t servo_angle;

	uint32_t start_ticks;
	uint32_t curr_time;
	uint16_t start_count;

public:
	sort_task(void);
	sort_task(servo_driver servo,
			UART_HandleTypeDef* huart);

	void run(void);

	int32_t get_duty(void);
};



#endif /* sorting_task_TASK_H__ */
