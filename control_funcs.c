/**
 * @file control_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Control functions source file.
 *
 * This file contains functions regarding the control algorithm.
 */

# include <math.h>
# include "master_header.h"
# include "control_header.h"

float VALVE__MAX_THRUST=0.5; ///< Maximum thrust of RCS solenoid valves (i.e. when fully opened) // TODO: confirm with Gautier!

/**
 * @fn void Fpitch_loop_control_setup()
 * This function setups up all control parameters relating to the pitch control.
 */
void Fpitch_loop_control_setup() {
	Fpitch_loop.satur = VALVE__MAX_THRUST;
	Fpitch_loop.control_range = 20*M_PI/180; // [rad]
	Fpitch_loop.K = Fpitch_loop.satur/Fpitch_loop.control_range;
	Fpitch_loop.Td = 0.7;
}

/**
 * @fn void Fyaw_loop_control_setup()
 * This function sets up all control parameters relating to the yaw control.
 */
void Fyaw_loop_control_setup() {
	Fyaw_loop.satur = VALVE__MAX_THRUST;
	Fyaw_loop.control_range = 20*M_PI/180; // [rad]
	Fyaw_loop.K = Fyaw_loop.satur/Fyaw_loop.control_range;
	Fyaw_loop.Td = 0.7;
}

/**
 * @fn void Mroll_loop_control_setup()
 * This function sets up all control parameters relating to the roll control.
 */
void Mroll_loop_control_setup() {
	Mroll_loop.satur = 2*d*VALVE__MAX_THRUST;
	Mroll_loop.control_range = 100*M_PI/180; // [rad/s]
	Mroll_loop.K = Mroll_loop.satur/Mroll_loop.control_range;
	Mroll_loop.Td = 0; // We do not use a derivative term in roll rate control
}
