/*
 * sorting_task/h
 *
 *  Created on: Jun 13, 2023
 *      Author: caleb erlenborn
 */

#ifndef sorting_task_TASK_H_
#define sorting_task_TASK_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "BNO055.h"
#include "motor_driver.h"
#include "controller_driver.h"
#include "servo_driver.h"
#include "encoder_driver.h"

#define rgb_addr      0x29<<1
#define clrdata_addr 0x94
#define enable_addr   0x80
#define status_addr   0x93
#define timing_addr   0x81
#define wait_addr     0x82
#define clr_addr      0x94
#define red_addr      0x96
#define green_addr    0x98
#define blue_addr     0x98


//! Class for sorting picked up balls
/*!
 * This class is used to implement the finite state machine for sorting ping-pong balls based
 * on their color. This FSM has 4 states: state 0 initialization, state 1 get color sensor output, state 2 rotate servo
 * motor 90 degrees. to the left if the ball is the correct color, state 3 move the servo motor to the right 90 degrees if the ball
 * is the wrong color.
 */
class sort_task
{
private:
	//* Creates a function for calling the specific state function
	/*!
	 * This function is used to call the required state function. It works by taking in the current
	 * state and calling the specific function corresponding to that state. The list of states and corresponding functions
	 * is kept in a struct that the function accesses.
	 */
	typedef void (sort_task::*state_fcn)(void);

	//! Current state
	uint32_t state; // Current state

	//! Private Variable for number of times the task has ran
	/*!
	 * This variable is incremented every time the task is ran.
	 */
	uint32_t runs; // Number of times Task Has Ran

	//! Private variable for the number of states in the FSM
	/*!
	 * Initialized to 4. Used to set the size of the struct containing each state function.
	 */
	uint32_t num_states; // Number of states

	//! Pointer to a list of all states
	/*!
	 * This is used in the state_fcn function to list the states and the corresponding function
	 */
	state_fcn* state_list; // Pointer to a List of States

	//! Internal function for state 0
	/*!
	 * Initialization state. In this state the I2C bus is initialized and set up for communication with the RGB color sensor.
	 */
	void state_0(void); // Initialize state

	//! Internal function for state 1: Read color sensor
	/*!
	 * In state one the color sensor reading is updated by reading from the specific register
	 * using the inputed I2C bus. This outputs the red green and white values. These values are then compared to
	 * a standard value corresponding to a red ball. If the values are not close enough to the desired value, set state to state 3.
	 * If the read values are close to the set value, set state to 2.
	 */
	void state_1(void); // Wait for color state

	//! Internal function for state 2: Move servo to accept
	/*!
	 * This state sets the servo to move to 90 degrees by using the passed through servo object's set position
	 * method.
	 */
	void state_2(void); // Set change servo to accept

	//! Internal function for state 3: Move servo to reject
	/*!
	 * This state sets the servo to move to 90 degrees by using the passed through object's set position method.
	 */
	void state_3(void);

	//! Servo object passed into the task object
	/*!
	 * The servo is used to put the balls in the correct location
	 */
	servo_driver servo;
	//!Pre configured Uart channel for debugging
	UART_HandleTypeDef * huart;

	//!Private variable corresponding to the servo set angle
	int16_t servo_angle;

	//! Private variable for the start time for each state
	uint32_t start_ticks;
	//! Private variable for the current time in the state
	uint32_t curr_time;
	uint16_t start_count;

	//! Pre-Initialized I2C pointer for RGB sensor communication
	I2C_HandleTypeDef* hi2c;

	//! Private variables for the red, green, and blue color sensor data
	uint16_t red_val,green_val,blue_val;

public:
	//! Prototype Default constructor
	sort_task(void);

	//! Constructor for the task creation
	/*!
	 * Only contains pre constructed object and configured objects
	 */
	sort_task(servo_driver servo,
			UART_HandleTypeDef* huart,
			I2C_HandleTypeDef* hi2c);

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



#endif /* sorting_task_TASK_H__ */
