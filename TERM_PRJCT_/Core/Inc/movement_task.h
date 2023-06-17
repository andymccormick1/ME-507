/*
 * movement_task.h
 *
 *  Created on: Jun 13, 2023
 *      Author: andymccormick
 */

#ifndef INC_MOVEMENT_TASK_H_
#define INC_MOVEMENT_TASK_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "BNO055.h"
#include "motor_driver.h"
#include "controller_driver.h"
#include "servo_driver.h"
#include "encoder_driver.h"

//! Class for the robot movement task
/*!
 * The class below is used to implement a finite state machine for the movement of the robot
 * about the game board. This consists of 5 states: an initialization state, state 1 move out of home box,
 * state 2 rotate to begin circle, state 3 move in circles around the center of the board, state 4 return to home.
 * This is implemented by switching between tasks when encoder values are reached and home position is reached.
 * Each state uses logic to stop motor motion if the deadman's switch is unpressed.
 */
class movement_task
{
private:
	//* Creates a function for calling the specific state function
	/*!
	 * This function is used to call the required state function. It works by taking in the current
	 * state and calling the specific function corresponding to that state. The list of states and corresponding functions
	 * is kept in a struct that the function accesses.
	 */
	typedef void (movement_task::*state_fcn)(void);

	//! Current State
	uint32_t state; // Current state

	//! Private Variable for number of times the task has ran
	/*!
	 * This variable is incremented every time the task is ran.
	 */
	uint32_t runs; // Number of times Task Has Ran

	//! Pointer to a list of all states
	/*!
	 * This is used in the state_fcn function to list the states and the corresponding function
	 */
	uint32_t num_states; // Number of states

	//! Internal function for state 0
	/*!
	 * Initialization state. In this state the I2C bus is initialized and set up for communication with the RGB color sensor.
	 */
	state_fcn* state_list; // Pointer to a List of States


	//! Initialization Task
	/*!
	 * This state always transitions to state 1 and sets all private variable values to zero.
	 */
	void state_0(void); // Initialize state

	//! State 1: Move out of Home
	/*!
	 * In this state the robot moves both wheels a specific amount of encoder ticks. The amount
	 * of encoder ticks corresponds to a distance where the robot is fully out of the home bounding box which was determined
	 * by testing. The movment is accomplished by using the motor driver objects set PWM function, the encoder get count method, and the
	 * feedback controller control function.
	 */
	void state_1(void); // Move Out of Home State

	//! Satte 2: Rotate to begin circle
	/*!
	 * In this state the robot moves both wheels a specific amount of encoder ticks. The amount
	 * of encoder ticks corresponds to turning the robot to be tangent to the circle The movment is accomplished by using the motor driver objects set PWM function, the encoder get count method, and the
	 * feedback controller control function.
	 */
	void state_2(void); // Move Turn To move in Circle State

	//! State 3: Move around in circle
	/*!
	 * In this state the robot moves around the game board in circles. The movement is accomplished by setting the left
	 * motor duty cycle to double that of the right motor. The difference in motor speeds results in the robot
	 * moving in a circular path. The movement is accomplished by using the motor set pwm function.
	 */
	void state_3(void); // Move in Circle State

	//! State 4: Return Home
	/*!
	 * This function is accomplished by using the imu heading and integrated current position. This
	 * state has not been fully implemented yet. It will likely require an additional state or task dedicated
	 * to determining the robot's position in the arena using the IMU heading and the encoder position.
	 */
	void state_4(void); // Return Home State

	//! Left rear motor object used for actuation
	motor_driver Left_Mot;
	//! Right rear motor object used for actuation
	motor_driver Right_Mot;
	//! Left motor encoder object used to get current encoder values and control position
	encoder_reader Left_Encoder;
	//! Right motor encoder object used to get current encoder values and control position
	encoder_reader Right_Encoder;
	//! Feedback controller used to control position of the left motor
	feedback_controller Left_Feedback;
	//! Feedback controller used to control position of the right motor
	feedback_controller Right_Feedback;

	//! Pointer to a pre-initialized uart object
	/*!
	 * Used for debugging
	 */
	UART_HandleTypeDef * huart;

	//! Right Motor set point for position control
	int32_t right_sp;
	//! Left motor set point for position control
	int32_t left_sp;

	//! Duty cycle for the right motor
	int16_t right_duty;
	//! Duty cycle for the left motor
	int16_t left_duty;

	//! Private variable for the start time for each state
	uint32_t start_ticks;
	//! Private variable for the current time in the state
	uint32_t curr_time;
	uint16_t start_count;

public:
	//! Prototype Default Constructor
	movement_task(void);

	//! Initialization For all task constructors
	/*!
	 * All the input arguments require pre-constructed and pre-configured. The uart pointer required is included purely
	 * for debugging purposes.
	 */
	movement_task(motor_driver Left_Mot,
			motor_driver Right_Mot,
			encoder_reader Left_Encoder,
			encoder_reader Right_Encoder,
			feedback_controller Left_Feedback,
			feedback_controller Right_Feedback,
			UART_HandleTypeDef* huart);

	//! Run method to implement the FSM
	/*!
	 * The run method uses the state variable to call the respective function using the state_fcn function.
	 *
	 */
	void run(void);

	//! Function to get the current duty output
	/*!
	 * This function is used to verify the pwm output before implementing it with the servo motor.
	 */
	int32_t get_duty(void);
};



#endif /* INC_MOVEMENT_TASK_H_ */
