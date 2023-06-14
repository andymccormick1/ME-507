/*
 * BNO055.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: andymccormick
 */

#include "BNO055.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

BNO055_imu::BNO055_imu(void)
{

}

// Initialize the IMU
BNO055_imu::BNO055_imu(I2C_HandleTypeDef* _hi2c)
						:hi2c(_hi2c)
{
	hi2c = _hi2c;
	uint8_t imu_mode = 0b00001000; // Operating Mode Register: IMU Mode
	uint8_t pwr_mode = 0b00000000; // Set to Normal Power Mode
	uint8_t unit_sel = 0b00000000; // Select Desired Units
	state = 0;
	uint8_t cur_opmode[1] = {0};
	while (state == 0)
	{
		if (HAL_I2C_IsDeviceReady(hi2c, imuaddr, 1, HAL_MAX_DELAY) == HAL_OK)
		{
			HAL_I2C_Mem_Write(hi2c, imuaddr, oprmode_addr, 1, &imu_mode, 1, HAL_MAX_DELAY);
			HAL_I2C_Mem_Write(hi2c, imuaddr, pwrmode_addr, 1, &pwr_mode, 1, HAL_MAX_DELAY);
			HAL_I2C_Mem_Write(hi2c, imuaddr, unit_sel_addr, 1, &unit_sel, 1, HAL_MAX_DELAY);
		}
		HAL_I2C_Mem_Read(hi2c, imuaddr, oprmode_addr, 1, cur_opmode, 1, HAL_MAX_DELAY);
		if (cur_opmode[0] == imu_mode)
		{
			state = 1;
		}

	}


}

uint16_t BNO055_imu::update_heading(void)
{
	HAL_I2C_Mem_Read(hi2c, imuaddr, heading_addr, 1, heading_buffer, 2, HAL_MAX_DELAY);
	cur_heading = (uint16_t)(heading_buffer[1]<<8 | heading_buffer[0]);
	heading_deg = cur_heading/16;
	return cur_heading;
}

uint8_t BNO055_imu::get_heading(void)
{
	return cur_heading;
}

void BNO055_imu::zero_heading(void)
{
	uint8_t configmode = 0b00000000;
	uint8_t imumode = 0b00001000;
	HAL_I2C_Mem_Write(hi2c, imuaddr, oprmode_addr, 1, &configmode, 1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Write(hi2c, imuaddr, oprmode_addr, 1, &imumode, 1, HAL_MAX_DELAY);
}

uint8_t BNO055_imu::calibrate(void)
{
	HAL_I2C_Mem_Read(hi2c, imuaddr, calib_addr, 1, calib_buffer, 1, HAL_MAX_DELAY);
	cur_calib = calib_buffer[0];

}

uint8_t BNO055_imu::get_calibration(void)
{
	return cur_calib;
}

uint8_t BNO055_imu::get_state(void)
{
	return state;
}
