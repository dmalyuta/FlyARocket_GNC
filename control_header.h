/**
 * @file control_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Control header file.
 *
 * This is the header to control_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef CONTROL_HEADER_H_
#define CONTROL_HEADER_H_

/**
 * @struct Control_loop
 * This structure holds all of the variables relating to a control loop of the GNC
 * algorithm. Namely, there are three such control loops used:
 * 	- 	For the pitch force, Fpitch
 * 	- 	For the yaw force, Fyaw
 * 	- 	For the roll moment, Mroll
 *
 * The combination of these three control loops stabilizes the rocket to point vertically up at all time
 */
struct Control_loop {
	double K; ///< Proportional term coefficient
	double Td; ///< Derivative term coefficient
	double satur; ///< Absolute ceiling of possible control loop output value
	double control_range; ///< At what angle from the vertical orientation to we begin applying maximum control input?
};

/**
 * @name Control loop group
 * These structures define fully the control of the rocket - i.e. the control gains.
 * @{
 */
struct Control_loop Fpitch_loop; ///< Pitch control loop, uses feedback on #theta_filt to tell what pitching corrective force we need
struct Control_loop Fyaw_loop; ///< Yaw control loop, uses feedback on #psi_filt to tell what yawing corrective force we need
struct Control_loop Mroll_loop; ///< Roll control loop, uses feedback on #phi_dot_filt to tell what corrective rolling moment we need
/** @} */

/** @cond INCLUDE_WITH_DOXYGEN */
void Fpitch_loop_control_setup();
void Fyaw_loop_control_setup();
void Mroll_loop_control_setup();
/** @endcond */

#endif /* CONTROL_HEADER_H_ */
