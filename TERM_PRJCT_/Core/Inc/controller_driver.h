/*
 * controller_driver.h
 *
 *  Created on: Jun 16, 2023
 *      Author: andymccormick
 */

#ifndef INC_CONTROLLER_DRIVER_H_
#define INC_CONTROLLER_DRIVER_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>     // Allows use of standard integers
#include <string.h>     // Allows use of string ops like strcpy
#include <cstdlib>		// allows use of functions like abs() for absolute value

//! Feedback controller class implements a closed loop KI controller
/*!
 * The feedback controller class simply provide Proportional and integral control to any system with accessible data.
 * The construction of these objects needs a timer and relevant channel. Usage requires the user to input a set point
 * and provide the current "position." The control algorithm then computes the error between the set point and current position.
 * The error is then scaled by the Kp value and integrated over time base on the Ki value.
 */
class feedback_controller{
	// Private variables
private:
	//! Private variable corresponding to the proportional gain constant
	/*!
	 * The Kp value scales the proportional gain accordingly.
	 */
	int16_t 		KP;			// Porportional gain (Kp)

	//! Private variable corresponding to the integral control constant
	/*!
	 * The Ki value scales the integral control accordingly.
	 */
	int16_t			KI;			// Integral gain (Ki)

	//! Private variable for the required motor duty cycle
	/*
	 * The control function outputs the required duty cycle stored in this variable. The duty
	 * cycle can then be sent to the motor driver object to set the new pwm
	 */
	int16_t		 	DUTY_CYCLE;	// Duty Cycle

	//! Private variable corresponding to the user defined set point
	/*!
	 * The set point is the desired position of the encoder. It is set manually and initialized as 0.
	 */
	int32_t		SET_POINT;	// Current Set Point

	//! Private Variable for the current encoder value
	/*!
	 * The current count is compared to the set point to compute the error. This value is obtained
	 * by using an encoder object to get the current count and pass that vaue into the controller object.
	 */
	int32_t		CURR_CNT;	// Current encoder position

	//! Private Variable for the accumulated Integral Error
	/*!
	 * Over time the integral of error is calculated and stored in this variable. This is computed by measuring the error
	 * and multiplying it by the time between measurements and scaling by Ki.
	 */
	int16_t		INT_ERR;	// integrated error

public:
	//! Constructor for the controller object
	/*!
	 * The controller object requires no parameters for construction. It is initialized without setting a set point.
	 */
	feedback_controller(void);				// prototype for default constructor

	//! Function to set the set point
	/*!
	 * Calling this function puts the data in the input parameter, SET_POINT, in the set point
	 * private variable.
	 */
	void set_setpoint(int32_t SET_POINT);	// prototype for method to set the encoder position setpoint

	//! Function to set the Kp gain value.
	/*!
	 * Sets the Kp private variable to the input argument. Kp is initialized as 0.
	 */
	void set_KP(int16_t KP);				// prototype for method to set KP

	//! Function to set the Ki integral gain
	/*!
	 * Sets the Ki private variable to the input argument. Ki is initialized as 0.
	 */
	void set_KI(int16_t KI);				// prototype for method to set KI

	//! Function to run the controller
	/*!
	 * When calling this function, the controller runs. If the Ki, Kp, and setpoint values have not been set, the controller will
	 * run with 0 values set. The function takes in the current encoder value to compare to set point. The encoder value would be
	 * obtained by reading the encoder using an encoder object.
	 */
	int16_t run(int32_t CURR_CNT);			// prototype for method to return duty cycle for given current encoder position

};


#endif /* INC_CONTROLLER_DRIVER_H_ */
