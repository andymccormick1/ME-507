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


// Class Motor Driver used to control the PWM signal for the duty cycle of a DC motor
class motor_driver {
// Private attributes and functions can only be accessed from within the class
private:
	int16_t	Duty_Cycle; 	// signed integer from representing the current duty cycle
	uint32_t ch_a ;			// timer channel for pwm pin 1
	uint32_t ch_b;			// timer channel for pwm pin 2
	TIM_HandleTypeDef* htim;	// timer channel handle

// Public attributes and functions can be used anywhere that has access to
// the motor driver object
public:
	motor_driver(void);								// Prototype for default constructor
	motor_driver(TIM_HandleTypeDef* htim,           // Prototype for initializing constructor
        		uint32_t ch_a,
				uint32_t ch_b);
    void Set_PWM(int16_t	Duty_Cycle);        // Prototype for set pwm method
    void Stop_Motor(void);  					// Prototype for disable motor method
};

// Encoder Reader class used to read  and return current encoder position
class encoder_reader {
// Private Variables can be accessed only from within the class
private:
	int32_t COUNT;				// Signed integer for the current value of encoder
	uint32_t prev_value;		// Unsigned integer for the counter value at the previous read time
	int32_t	DELTA;				// Signed integer for the change in counter value
	TIM_HandleTypeDef* htim;	// timer channel handle

public:
	encoder_reader(void);						// prototype for default constructor
	encoder_reader(TIM_HandleTypeDef* htim);	// prototype for initializing constructor
	void zero_count(void);						// prototype for method to zero the current encoder value
	int32_t get_count(void);					// prototype for method to return the current encoder value

};


// Feedback controller class implements a closed loop KI controller
class feedback_controller{
	// Private variables
private:
	int16_t 		KP;			// Porportional gain (Kp)
	int16_t			KI;			// Integral gain (Ki)
	int16_t		 	DUTY_CYCLE;	// Duty Cycle
	int32_t		SET_POINT;	// Current Set Point
	int32_t		CURR_CNT;	// Current encoder position
	int16_t		INT_ERR;	// integrated error

public:
	feedback_controller(void);				// prototype for default constructor
	void set_setpoint(int32_t SET_POINT);	// prototype for method to set the encoder position setpoint
	void set_KP(int16_t KP);				// prototype for method to set KP
	void set_KI(int16_t KI);				// prototype for method to set KI
	int16_t run(int32_t CURR_CNT);			// prototype for method to return duty cycle for given current encoder position

};
#endif /* INC_MOTOR_DRIVER_H_ */
