/*
 * BNO055.h
 *
 *  Created on: Jun 13, 2023
 *      Author: andymccormick
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>     // Allows use of standard integers
#include <string.h>     // Allows use of string ops like strcpy
#include <cstdlib>		// allows use of functions like abs() for absolute value


// Relevant Addresses for Setup
#define imuaddr                 0x28<<1
#define oprmode_addr            0x3D
#define heading_addr            0x1E
#define heading_addr_msb        0x1F
#define calib_addr              0x35
#define pwrmode_addr            0x3E
#define unit_sel_addr        0x3B


// Register Map
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00        // value: 0xA0
#define BNO055_ACC_ID 0x01         // value: 0xFB
#define BNO055_MAG_ID 0x02         // value: 0x32
#define BNO055_GYRO_ID 0x03        // value: 0x0F
#define BNO055_SW_REV_ID_LSB 0x04  // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05  // value: 0x03
#define BNO055_BL_REV_ID 0x06      // N/A
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

//! A Class for the Use of the Adafruit BNO055 IMU

/*!
 * The BNO0855_imu class is used to communicate with the adafruit BNO055 imu over the I2C bus.
 * This calss specifically obtains the heading value from the specific register on the imu. Although there are
 * many additional measurements held on the imu, the heading is the only one accessed because it is the only measurement needed.
 * Other measurements can easily be obtained by reading other registers.
 */

class BNO055_imu {


private:

	//! A Buffer for the calibration data
	/*!
	 * The calibration data is stored in the register with address 0x35. When this register is read in the setup method,
	 * the current value is out in this buffer.
	 */
	uint8_t calib_buffer[1];

	//! Private variable for the actual calibration
	/*!
	 * Data from the buffer is transferred to this variable for easy access.
	 */
	uint8_t cur_calib;

	//!Private variable for the current heading
	/*!
	 * Data from the heading buffer is transferred to this variable as 16 bits for easy access and manipulation.
	 */
	uint16_t cur_heading;

	//! A private variable for the current heading in degrees.
	/*!
	 * The number stored in the cur_heading variable is converted to degrees and stored in this variable.
	 */
	uint8_t heading_deg;

	//! A 2 byte Buffer for the heading data.
	/*!
	 * Data is read from the heading register on the imu at address 0x1E. The 2 bytes (16 bits) are stored temporarily in the buffer.
	 */
	uint8_t heading_buffer[2];

	//! A private variable describing the state of the imu
	/*!
	 * The state is initialized as 0. Once communication has been established, the state changes to one.
	 * If communication is not established the program will be stuck in a while loop and print over uart the issue.
	 */
	uint8_t state;

	//! A Pointer to the I2C bus used for communication
	/*!
	 * The I2C bus must be initialized and configured in the main file before the IMU object is created.
	 * The I2C bus should be running no faster than 400 kHz, and all pins should be pulled up, preferably by external pull up resistors.
	 */
	I2C_HandleTypeDef* hi2c;



public:
	//! Initialization
	BNO055_imu(void);

	//! IMU initialization function
	/*!
	 * Takes an input of the pre-initizalized I2C bus. This function configures the IMU in IMU mode in order to allow
	 * fusion of data outputs. This also verifies the I2C bus is working and can communicate with the IMU. Additionally,
	 * the initialization hecks for sensor calibration before moving on, but has been removed due to implementation
	 * difficulties.
	 */
	BNO055_imu(I2C_HandleTypeDef* hi2c);

	//! returns the current heading in degrees.
	/*!
	 * This function does not update the heading, it merely returns the current value stored. This is not the msot useful function.
	 */
	uint8_t get_heading(void);

	//! Function to calibrate the IMU
	/*!
	 * This function can be used to verify the IMU is calibrated. It returns the value of the calibration register.
	 * It also et stuck in a while loop and outputs the error if the IMU is not calibrated. This should be used sparingly
	 * as it can stop the rest of the code from running.
	 */
	uint8_t calibrate(void);

	//! Function to get the current calibration status
	/*!
	 * This function purely returns the calibration status and nothing else. Can be used to verify the device is ready to output data.
	 */
	uint8_t get_calibration(void);

	//! Function to obtain the IMU state
	/*!
	 * This function was used to debug the I2C bus. A state of 0 means the IMU is not communicating while
	 * a state of 1 means the IMU is communicating.
	 */
	uint8_t get_state(void);

	//! Function zero the heading
	/*!
	 * This function is used to set a heading datum of heading equal to zero. This can be useful to set the devices
	 * home.
	 */
	void zero_heading(void);

	//! Function to update and return the current heading
	/*!
	 * This function reads the Heading register and puts the 2 bytes in the heading buffer. The 2 bytes are then
	 * put in the 16 bit heading file by shifting the MSB to the left by 8 and oring that value with the LSB.
	 * This final value is then divided by 16 to return the heading value in degrees.
	 */
	uint16_t update_heading(void);
};


#endif /* INC_BNO055_H_ */
