/*
 * servo_driver.h
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>     // Allows use of standard integers
#include <string.h>     // Allows use of string ops like strcpy
#include <cstdlib>		// allows use of functions like abs() for absolute value


//! Class Motor Driver used to control the PWM signal for the position of a servo motor
/*!
 * The servo driver class provides an interface to set the position of the servo motor. This is accomplished
 * by setting the duty cycle of a pwm timer. This timer must be configured before creating a servo driver and must be set to a
 * specific prescaler and ARR value. (95 and 19999 for current implementation)
 */
class servo_driver {
// Private attributes and functions can only be accessed from within the class
private:
	//! Private vairbale for the servo angle
	/*!
	 * Thsi variable corresponds to the current servo angle. This value will be used to determine
	 * the required pwm pulse width.
	 */
	int16_t	ANGLE; 				// signed integer from representing the current duty cycle

	//! Private variable for the channel the servo is running on
	/*!
	 * This variable is either 1, 2, 3, or 4 depending on what timer channel the servo control wire is
	 * connected to.
	 */
	uint32_t channel ;			// timer channel for pwm pin 1

	//! Pointer to a pre-initialized timer
	/*!
	 * This pointer corresponds to the timer configured to run pwm for the servo control. The timer must be carefully configured
	 * to deliver to deliver the correct period and pulse width to the servo. If these values are not correct the servo will
	 * not rotate.
	 */
	TIM_HandleTypeDef* htim;	// timer channel handle

// Public attributes and functions can be used anywhere that has access to
// the motor driver object
public:
	//! Prototype default constructor
	servo_driver(void);								// Prototype for default constructor

	//! Constructor for all servo objects
	/*!
	 * The servo driver is created with a timer and a channel. The corresponding output pin of the timer and channel
	 * connects directly to the servo command line.
	 */
	servo_driver(TIM_HandleTypeDef* htim,           // Prototype for initializing constructor
        		uint32_t channel);

	//! Function to set the servo position
	/*!
	 * The set position takes the desired position in degrees as an input and converts the degree value desired to
	 * a necessary pulse width for servo control. Theposition saturates and +/- 90 degrees. In order to calcualte the pulse with, the degree
	 * value is multiplied by 500 and divided by 90. This value is either added to or subtracted from 1500 depending on if the desired
	 * position is positive or negative. A pulse width of 1500 corresponds to an angle of 0. The timer channel is then
	 * set to produce a waveform with the desired characteristics.
	 */
    void Set_Position(int16_t	ANGLE);        // Prototype for set pwm method
};


#endif /* INC_SERVO_DRIVER_H_ */
