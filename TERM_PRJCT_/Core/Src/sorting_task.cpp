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
#include "sorting_task.h"

sort_task::sort_task(servo_driver _servo,
			UART_HandleTypeDef* _huart,
			I2C_HandleTypeDef* _hi2c)
				:state(0), runs(0),
				state_list(new state_fcn[3]{(sort_task::state_fcn) &sort_task::state_0,
						(sort_task::state_fcn) &sort_task::state_1,
						(sort_task::state_fcn) &sort_task::state_2}),
						num_states(3),
				servo(_servo),
				servo_angle(0),
				huart(_huart),
				start_ticks(0),
				curr_time(0),
				start_count(0),
				hi2c(_hi2c),
				red_val(0),
				green_val(0),
				blue_val(0)
{

}

void sort_task::run(void)
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

void sort_task::state_0(void)
{
	// Initialization state
	// Set servo to zero
	uint8_t enable_set = 0b00001011;
	uint8_t timing_set = 0xD5;
	uint8_t wait_set = 0x8B;
	HAL_I2C_Mem_Write(hi2c, rgb_addr, enable_addr, 1, &enable_set, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(hi2c, rgb_addr, timing_addr, 1, &timing_set, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(hi2c, rgb_addr, wait_addr, 1, &wait_set, 1, HAL_MAX_DELAY);

	state = 1;
	char buffer[50] = {0};
	int32_t s4 = sprintf(buffer, "Task 2 State 0 \r\n");
	HAL_UART_Transmit(huart,(uint8_t*) buffer, s4, 1000);
}

void sort_task::state_1(void)
{
	// Sort the ball from RGB data
	uint8_t red_buff[2], green_buff[2], blue_buff[2];
	HAL_I2C_Mem_Read(hi2c, rgb_addr, red_addr, 1, red_buff, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(hi2c, rgb_addr, green_addr, 1, green_buff, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(hi2c, rgb_addr, blue_addr, 1, blue_buff, 2, HAL_MAX_DELAY);
	red_val =(uint16_t)(red_buff[1]<<8 | red_buff[0]);
	green_val =(uint16_t)(green_buff[1]<<8 | green_buff[0]);
	blue_val =(uint16_t)(blue_buff[1]<<8 | blue_buff[0]);

	start_ticks = 1;
	servo.Set_Position(servo_angle);

	char buffer[50] = {0};
	int32_t s4 = sprintf(buffer, "Red: %ld Green: %ld Blue: %ld \r\n", red_val, green_val, blue_val);
	HAL_UART_Transmit(huart,(uint8_t*) buffer, s4, 1000);

	if ((red_val > 50) && (green_val < 50) && (blue_val < 50))
	{
		servo_angle = 90;
		state = 2;
	}
	HAL_Delay(100);


}

void sort_task::state_2(void)
{
	if (start_ticks == 1){
		start_count =  HAL_GetTick();
		start_ticks = 0;
	}
	curr_time = HAL_GetTick() - start_count;	// elapsed time in the current state
	servo.Set_Position(servo_angle);

	if (curr_time > 2000){		// Transition to sort balls after 2 seconds and return to 0 degree position
		state = 1;
		servo_angle = 0;
		servo.Set_Position(servo_angle);
	}

}
