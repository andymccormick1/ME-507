/*
 * motor_driver.h
 *
 *  Created on: May 1, 2023
 *      Author: caleb erlenborn
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>     // Allows use of standard integers
#include <string.h>     // Allows use of string ops like strcpy
#include <cstdlib>		// allows use of functions like abs() for absolute value


//! Class Motor Driver used to control the PWM signal for the duty cycle of a DC motor
/*!
 * The motor driver class controls a DC motor by altering the pwm signal sent to the motor driver.
 * A motor driver object is created simply with a pointer to an pre-initialized timer, and two channels
 * connected to the motor driver.
 */
class motor_driver {
// Private attributes and functions can only be accessed from within the class
private:
	//! Private duty cycle variable
	/*!
	 * The duty cycle variable simply contains the duty cycle to be sent to the motor driver. The maximum duty cycle is
	 * 5000 which corresponds to a 50% duty cycle. Any number above 5000 saturates to 5000 in order to stop the
	 * robot from moving too quickly.
	 */
	int16_t	Duty_Cycle; 	// signed integer from representing the current duty cycle

	//! Private variable for one of the timer channels running the motor
	/*!
	 * This variable is a 32 bit unsigned integer and is simply a 1, 2, 3, 4. Care must be taken to connect the motor to
	 * the correct pins corresponding to these two channels.
	 */
	uint32_t ch_a ;			// timer channel for pwm pin 1

	//! Private variable for one of the timer channels running the motor
	/*!
	 * This variable is a 32 bit unsigned integer and is simply a 1, 2, 3, 4. Care must be taken to connect the motor to
	 * the correct pins corresponding to these two channels.
	 */
	uint32_t ch_b;			// timer channel for pwm pin 2

	//! Pointer to a pre-initialized timer
	/*!
	 * This is a pointer to a specific timer which has been pre-configured for pwm output.
	 * The timer should be configured with zero prescaler and an auto-reload value of 4899. (or whatever value sets
	 * the period of the timer to 50 ms)
	 */
	TIM_HandleTypeDef* htim;	// timer channel handle

// Public attributes and functions can be used anywhere that has access to
// the motor driver object
public:

	//! Prototype for the default constructor
	motor_driver(void);								// Prototype for default constructor

	//! Motor driver initialization
	/*!
	 * Requires an initialized timer, and two corresponding channels used for motor pwm.
	 */
	motor_driver(TIM_HandleTypeDef* htim,           // Prototype for initializing constructor
        		uint32_t ch_a,
				uint32_t ch_b);
	//! Function to set the motors pwm signal
	/*!
	 * This function sets the duty cycle variable to this functions parameter input, Duty_Cycle. This value is then saturated to 5000
	 * or -5000 if the input value is larger or smaller.
	 * Then, based on if the set value is positive or negative, one channel is set to generate a pulse of the inputted duty cycle,
	 * while the other channel is set to low.
	 */
    void Set_PWM(int16_t	Duty_Cycle);        // Prototype for set pwm method

    //! Function to stop the motor
    /*!
     * This function simply stops pwm on the specified timer channels. If this function is used,
     * the motor cannot be turned on by setting the duty cycle.
     */
    void Stop_Motor(void);  					// Prototype for disable motor method
};





#endif /* INC_MOTOR_DRIVER_H_ */
