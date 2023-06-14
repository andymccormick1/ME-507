/*
 * Movement_task.cpp
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

movement_task::movement_task(BNO055_imu IMU,
		motor_driver _Left_Mot,
		motor_driver _Right_Mot,
		encoder_reader _Left_Encoder,
		encoder_reader _Right_Encoder,
		feedback_controller _Left_Feedback,
		feedback_controller _Right_Feedback)
				:state(0), runs(0),
				state_list(new state_fcn[5]{(movement_task::state_fcn) &movement_task::state_0,
						(movement_task::state_fcn) &movement_task::state_1,
						(movement_task::state_fcn) &movement_task::state_2,
						(movement_task::state_fcn) &movement_task::state_3,
						(movement_task::state_fcn) &movement_task::state_4}),
						num_states(5),
				Left_Mot(_Left_Mot),
				Right_Mot(_Right_Mot),
				Left_Encoder(_Left_Encoder),
				Right_Encoder(_Right_Encoder),
				Left_Feedback(_Left_Feedback),
				Right_Feedback(_Right_Feedback),
				right_sp(0),
				left_sp(0),
				right_duty(0),
				left_duty(0)
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
	// Maybe Calibrate IMU Here
	// Zero Everything
	Right_Encoder.zero_count();
	Left_Encoder.zero_count();
	right_sp = 0;
	left_sp = 0;
	right_duty = 0;
	left_duty = 0;
	state = 1;
}

void movement_task::state_1(void)
{
	// Move out of Home Area
	right_sp = 1000;
	left_sp = right_sp;

	int32_t right_count = Right_Encoder.get_count();
	int32_t left_count = Left_Encoder.get_count();
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	{


	Right_Feedback.set_setpoint(right_sp);
	Left_Feedback.set_setpoint(left_sp);

	right_duty = Right_Feedback.run(right_count);
	left_duty = Left_Feedback.run(left_count);

	Right_Mot.Set_PWM(right_duty);
	Left_Mot.Set_PWM(left_duty);
	} else
	{
		Right_Mot.Set_PWM(0);
		Left_Mot.Set_PWM(0);
	}

	if (((right_sp - right_count) < 10) && ((left_sp - left_count) < 10))
	{
		state = 2;
	}




}

void movement_task::state_2(void)
{
	// Rotate To Direction to Start Circle
	right_sp = 1000;
	left_sp = -1000;

	int32_t right_count = Right_Encoder.get_count();
	int32_t left_count = Left_Encoder.get_count();
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	{


	Right_Feedback.set_setpoint(right_sp);
	Left_Feedback.set_setpoint(left_sp);

	right_duty = Right_Feedback.run(right_count);
	left_duty = Left_Feedback.run(left_count);

	Right_Mot.Set_PWM(right_duty);
	Left_Mot.Set_PWM(left_duty);
	} else
	{
		Right_Mot.Set_PWM(0);
		Left_Mot.Set_PWM(0);
	}

	if (((right_sp - right_count) < 10) && ((left_sp - left_count) < 10))
	{
		state = 4;
	}

}

void movement_task::state_3(void)
{
	// Drive In Circle
	state = 4;
}

void movement_task::state_4(void)
{
	// Return Home
	state = 1;
}

int32_t movement_task::get_duty(void)
{
	return Left_Encoder.get_count();
}
