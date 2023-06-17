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
#include "controller_driver.h"
#include "servo_driver.h"
#include "encoder_driver.h"

movement_task::movement_task(motor_driver _Left_Mot,
		motor_driver _Right_Mot,
		encoder_reader _Left_Encoder,
		encoder_reader _Right_Encoder,
		feedback_controller _Left_Feedback,
		feedback_controller _Right_Feedback,
		UART_HandleTypeDef* _huart)
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
				left_duty(0),
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
	right_sp = 650;
	left_sp = right_sp;

	Left_Feedback.set_setpoint(left_sp);
	Right_Feedback.set_setpoint(right_sp);



	char buffer[50] = {0};
	int32_t right_count = Right_Encoder.get_count();
	int32_t left_count = Left_Encoder.get_count();

	int32_t n3 = sprintf(buffer, "Right Count: %d  ",right_count);
	HAL_UART_Transmit(huart,(uint8_t*) buffer, n3, 1000);
	int32_t n2 = sprintf(buffer, "Left Count: %d \r\n",left_count);
	HAL_UART_Transmit(huart,(uint8_t*) buffer, n2, 1000);



	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	{
		if (start_count == 0)
		{
			start_ticks = HAL_GetTick();
			start_count = 1;
		}
		curr_time = HAL_GetTick() - start_ticks;

		int32_t n1 = sprintf(buffer, "BUTTON PRESSED  \r\n");
		HAL_UART_Transmit(huart,(uint8_t*) buffer, n1, 1000);



		right_duty = Right_Feedback.run(right_count);
		left_duty = Left_Feedback.run(left_count);

		if (right_duty > 2500){
			right_duty = 2500;
		}
		else if (right_duty < -2500){
			right_duty = -2500;
		}

		if (left_duty > 2500){
			left_duty = 2500;
		}
		else if (left_duty < -2500){
			left_duty = -2500;
		}



	} else
	{
		char buffer[50] = {0};
		int32_t n1 = sprintf(buffer, "BUTTON Not PRESSED  \r\n");
		//HAL_UART_Transmit(huart,(uint8_t*) buffer, n1, 1000);
		right_duty = 0;
		left_duty = 0;

	}
	Right_Mot.Set_PWM(right_duty);
	Left_Mot.Set_PWM(left_duty);



	if (curr_time > 5000)
	{
		state = 2;
		start_count = 0;
	}

}

void movement_task::state_2(void)
{
	// Move out of Home Area
	right_sp = 896;
	left_sp = 395;

	Left_Feedback.set_setpoint(left_sp);
	Right_Feedback.set_setpoint(right_sp);



	char buffer[50] = {0};
	int32_t right_count = Right_Encoder.get_count();
	int32_t left_count = Left_Encoder.get_count();

	int32_t n3 = sprintf(buffer, "Right Count: %d  ",right_count);
	HAL_UART_Transmit(huart,(uint8_t*) buffer, n3, 1000);
	int32_t n2 = sprintf(buffer, "Left Count: %d \r\n",left_count);
	HAL_UART_Transmit(huart,(uint8_t*) buffer, n2, 1000);



	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	{
		if (start_count == 0)
		{
			start_ticks = HAL_GetTick();
			start_count = 1;
		}
		curr_time = HAL_GetTick() - start_ticks;

		int32_t n1 = sprintf(buffer, "BUTTON PRESSED  \r\n");
		HAL_UART_Transmit(huart,(uint8_t*) buffer, n1, 1000);



		right_duty = Right_Feedback.run(right_count);
		left_duty = Left_Feedback.run(left_count);

		if (right_duty > 2500){
			right_duty = 2500;
		}
		else if (right_duty < -2500){
			right_duty = -2500;
		}

		if (left_duty > 2500){
			left_duty = 2500;
		}
		else if (left_duty < -2500){
			left_duty = -2500;
		}



	} else
	{
		char buffer[50] = {0};
		int32_t n1 = sprintf(buffer, "BUTTON Not PRESSED  \r\n");
		//HAL_UART_Transmit(huart,(uint8_t*) buffer, n1, 1000);
		right_duty = 0;
		left_duty = 0;

	}
	Right_Mot.Set_PWM(right_duty);
	Left_Mot.Set_PWM(left_duty);



	if (curr_time > 5000)
	{
		state = 3;
		start_count = 0;
	}

}

void movement_task::state_3(void)
{
	// Drive In Circle


	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
	{
		if (start_count == 0)
		{
			start_ticks = HAL_GetTick();
			start_count = 1;
		}
		curr_time = HAL_GetTick() - start_ticks;
		right_duty = 1250;
		left_duty  = 2500;

	} else
	{
		right_duty = 0;
		left_duty = 0;
	}

	Right_Mot.Set_PWM(right_duty);
	Left_Mot.Set_PWM(left_duty);

	if (curr_time > 20000)
	{
		state = 4;
		start_count = 0;
	}
}

void movement_task::state_4(void)
{
	// Return Home
	Right_Mot.Set_PWM(0);
	Left_Mot.Set_PWM(0);
	char buffer[50] = {0};
	int32_t s4 = sprintf(buffer, "State 4\r\n");
	HAL_UART_Transmit(huart,(uint8_t*) buffer, s4, 1000);
}

int32_t movement_task::get_duty(void)
{
	return Left_Encoder.get_count();
}
