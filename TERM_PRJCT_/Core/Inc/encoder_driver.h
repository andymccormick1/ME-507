/*
 * encoder_driver.h
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#ifndef INC_ENCODER_DRIVER_H_
#define INC_ENCODER_DRIVER_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>     // Allows use of standard integers
#include <string.h>     // Allows use of string ops like strcpy
#include <cstdlib>		// allows use of functions like abs() for absolute value


//! Encoder Reader class used to read  and return current encoder position
/*!
 * The encoder class is used to read the current encoder value of a motor encoder. This requires a pre-initialized
 * timer configured to encoder mode. The encoder simply works by reading the current encoder value and finding the difference between this value
 * and the previous value. The difference is then added to the current count which contains the
 * position of the motor.
 */
class encoder_reader {
// Private Variables can be accessed only from within the class
private:

	//! Private Variable containing the current encoder value
	/*!
	 * This value is constantly updated by the difference in encoder ticks.
	 */
	int32_t COUNT;				// Signed integer for the current value of encoder

	//! Private Variable for the previous encoder value
	/*!
	 * This value is compared to the current value to get how far the encoder has moved between measurements
	 */
	uint32_t prev_value;		// Unsigned integer for the counter value at the previous read time

	//! Private variable for the difference between the current and previous encoder value.
	/*!
	 * This value is obtained by subtracting the previous value from the current value. DELTA is then added
	 * to the count to update the current position of the motor.
	 */
	int32_t	DELTA;				// Signed integer for the change in counter value

	//! Pointer to a pre-configured timer object
	/*!
	 * The timer should be initialized before the encoder object is created and should be configured in encoder mode.
	 * The timer does the bulk of the heavy lifting by keeping track of the encoder pulses coming voer the channel.
	 * The timer can then be "read" to obtain the current value.
	 */
	TIM_HandleTypeDef* htim;	// timer channel handle

public:
	//! Default constructor prototype
	encoder_reader(void);						// prototype for default constructor

	//! Constructor for encoder objects
	/*!
	 * The encoder object simply needs pointer to a timer to be initialized.
	 */
	encoder_reader(TIM_HandleTypeDef* htim);	// prototype for initializing constructor
	//! Function to zero the current motor position
	/*!
	 * This function sets the COUNT variable to zero. This can be done to set a home position or use a current position
	 * as a datum.
	 */
	void zero_count(void);						// prototype for method to zero the current encoder value
	//! Function to get the current count value
	/*!
	 * This function simply returns the value stored in the COUNT variable. Before returning the value, the COUNT is upodated.
	 * This is done by getting the timer (encoders) current value and subtracting the previous value. This value is then
	 * added to the COUNT value which is then returned.
	 */
	int32_t get_count(void);					// prototype for method to return the current encoder value

};

#endif /* INC_ENCODER_DRIVER_H_ */
