/*
 * movement_task.h
 *
 *  Created on: Jun 13, 2023
 *      Author: andymccormick
 */

#ifndef INC_MOVEMENT_TASK_H_
#define INC_MOVEMENT_TASK_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "BNO055.h"
#include "motor_driver.h"

class movement_task
{
private:

	typedef void (movement_task::*state_fcn)(void);

	uint32_t state; // Current state

	uint32_t runs; // Number of times Task Has Ran

	uint32_t num_states; // Number of states

	state_fcn* state_list; // Pointer to a List of States

	void state_0(void); // Initialize state
	void state_1(void); // Move Out of Home State
	void state_2(void); // Move Turn To move in Circle State
	void state_3(void); // Move in Circle State
	void state_4(void); // Return Home State

	BNO055_imu IMU;
	motor_driver Left_Mot;
	motor_driver Right_Mot;
	encoder_reader Left_Encoder;
	encoder_reader Right_Encoder;
	feedback_controller Left_Feedback;
	feedback_controller Right_Feedback;

	UART_HandleTypeDef * huart;

	int32_t right_sp;
	int32_t left_sp;

	int16_t right_duty;
	int16_t left_duty;

	uint32_t start_ticks;
	uint32_t curr_time;
	uint16_t start_count;

public:
	movement_task(void);
	movement_task(BNO055_imu IMU,
			motor_driver Left_Mot,
			motor_driver Right_Mot,
			encoder_reader Left_Encoder,
			encoder_reader Right_Encoder,
			feedback_controller Left_Feedback,
			feedback_controller Right_Feedback,
			UART_HandleTypeDef* huart);

	void run(void);

	int32_t get_duty(void);
};



#endif /* INC_MOVEMENT_TASK_H_ */
