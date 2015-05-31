/**
 * @file master_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Master header file.
 *
 * This is the header to master_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef MASTER_HEADER_H_
#define MASTER_HEADER_H_

# include <stdint.h>
# include <stdio.h>
# include <sys/time.h>
# include <pthread.h>
# include "la_header.h"

# define VALVE_CHARAC_RESOLUTION 13 ///< The number of points there are in the calibrated valve thrust curve (flow rate vs. PWM)

char ERROR_MESSAGE[200]; ///< Allocate buffer for an error message to be printed into #error_log if errors occur

/**
 * @name IMU timing
 * Contains the timing structures and variables necessary for setting the IMU filtering thread loop frequency (see get_filtered_attitude_parallel()).
 */
/** @{ */
struct timeval now_imu;
struct timeval before_imu;
struct timeval elapsed_imu;
unsigned long long int time_imu;
/** @} */

/**
 * @name Pressure sensor timing
 * Contains the timing structures and variables necessary to set the  the pressure/temperature logging thread loop frequency (see get_readings_SPI_parallel())
 */
/** @{ */
struct timeval now_pressure;
struct timeval before_pressure;
struct timeval elapsed_pressure;
unsigned long long int time_pressure;
/** @} */

/**
 * @name General loop timing
 * Contains the timing structures and variables necessary to make sure a loop executes a given amount of time
 */
/** @{ */
struct timeval now_loop;
struct timeval before_loop;
struct timeval elapsed_loop;
unsigned long long int time_loop;
/** @} */

/**
 * @name Control loop timing
 * Contains the timing structures and variables necessary for timing necessary to set the control loop frequency (see main())
 */
/** @{ */
struct timeval now_control;
struct timeval before_control;
struct timeval elapsed_control;
unsigned long long int time_control;
/** @} */

/**
 * @name IMU filter get global time
 * Contains the timing structures and variables necessary for getting the global time within the IMU data filtering loop (see get_filtered_attitude_parallel())
 */
/** @{ */
struct timeval now_imu_glob;
struct timeval elapsed_imu_glob;
unsigned long long int time_imu_glob;
/** @} */

/**
 * @name Pressure read get global time
 * Contains the timing structures and variables necessary for getting the global time within the pressure/tempearture sensor data logging loop (see get_readings_SPI_parallel())
 */
/** @{ */
struct timeval now_pressure_glob;
struct timeval elapsed_pressure_glob;
unsigned long long int time_pressure_glob;
/** @} */

/**
 * @name Control loop get global time
 * Contains the timing structures and variables necessary for getting the global time within the control loop (see main())
 */
/** @{ */
struct timeval now_control_glob;
struct timeval elapsed_control_glob;
unsigned long long int time_control_glob;
/** @} */

struct timeval GLOBAL__TIME_STARTPOINT; ///< Structure holding the time when the program started (very first line of main())

extern unsigned char IMU_SYNCHED;

extern unsigned long long int SPI__READ_TIMESTEP; ///< Time intervals [us] at which we read over SPI (for pressure/temperature Honeywell sensors).
extern unsigned char SPI_quit; ///< ==0 by default, ==1 signals the SPI reading thread (get_readings_SPI_parallel()) to exit.

extern unsigned long long int IMU__READ_TIMESTEP; ///< Time intervals [us] at which we read over UART the IMU data.
extern unsigned char IMU_quit; ///< ==0 by default, ==1 signals the IMU reading and filtering threads (read_IMU_parallel() and get_filtered_attitude_parallel()) to exit.

extern unsigned char PWM1; ///< PWM value for the R1 valve
extern unsigned char PWM2; ///< PWM value for the R2 valve
extern unsigned char PWM3; ///< PWM value for the R3 valve
extern unsigned char PWM4; ///< PWM value for the R4 valve
extern double R1; ///< Valve R1 thrust
extern double R2; ///< Valve R2 thrust
extern double R3; ///< Valve R3 thrust
extern double R4; ///< Valve R4 thrust

extern unsigned int PWM_valve_charac[VALVE_CHARAC_RESOLUTION];
extern double R_valve_charac[VALVE_CHARAC_RESOLUTION];

extern double d; ///< [m] offset distance of RCS valves from centerline (for roll control)
extern double Fpitch; ///< Pitch force (parallel to body -Z axis, so as to produce positive pitch rate when Fpitch>0 (right hand rule))
extern double Fyaw; ///< Yaw force (parallel to body +Y axis, so as to produce positive yaw rate when Fyaw>0 (right hand rule))
extern double Mroll; ///< Roll moment (positive about +X axis, so as to produce positive roll rate when Mroll>0 (right hand rule))
extern int N; ///< Number of variables in cost function. Our variables are R1, R2, R3, R4 so N=4
extern int M1; ///< No (<=) type constraints
extern int M2; ///< No (>=) type constraints
extern int M3; ///< 3 (=) type constraints (for Fpitch, Fyaw, Mroll)
extern int M; ///< Total number of constraints (M=M1+M2+M3)
extern float VALVE__MAX_THRUST; ///< Maximum thrust of RCS solenoid valves (i.e. when fully opened)

/**
 * @name Log files group
 * Contains the pointes to files we use for recording flight data.
 * @{
 */
extern FILE *error_log; ///< Error log (stores errors)
extern FILE *pressure_log; ///< Pressure log (stores pressures and tempeartures collected by Honeywell HSC TruStability sensors)
extern FILE *imu_log; ///< IMU log (stores raw and filtered Euler angles and angular rates, the body rates and the accelerometer data)
extern FILE *control_log; ///< Control log (stores the control loop data such as computed #Fpitch, #Fyaw, #Mroll, the optimally distributed valves thrusts #R1,...,#R4 and the computed PWM signals #PWM1,...,#PWM4)
/** @} */

char MESSAGE[700]; ///< Message buffer string sometimes used for putting together a string, then writing it to a file

pthread_mutex_t error_log_write_lock; ///< Mutex to protect multiple threads from writing to the error log file at once

/*********** FILTERING MATRICES ************
 * (Kalman filter)
 */

/**
 * @name Yaw Kalman filtering matrices
 * These matrices pertain to the real-time Kalman filtering of the yaw angle and angular rate
 * @{
 */
struct MATRIX P_psi; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #psi_filt estimate
struct MATRIX P_psidot; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #psi_dot_filt estimate
struct MATRIX x_psi; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #psi_filt)
struct MATRIX x_psidot; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #psi_dot_filt)
struct MATRIX Q_psi; ///< Covariance matrix of process noise of #psi
struct MATRIX Q_psidot; ///< Covariance matrix of process noise of #psi_dot
struct MATRIX R_psi; ///< Covariance matrix of observation of #psi
struct MATRIX R_psidot; ///< Covariance matrix of observation of #psi_dot
/** @} */

/**
 * @name Pitch Kalman filtering matrices
 * These matrices pertain to the real-time Kalman filtering of the pitch angle and angular rate
 * @{
 */
struct MATRIX P_theta; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #theta_filt estimate
struct MATRIX x_theta; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #theta_filt)
struct MATRIX Q_theta; ///< Covariance matrix of process noise of #theta
struct MATRIX R_theta; ///< Covariance matrix of observation of #theta
struct MATRIX P_thetadot; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #theta_dot_filt estimate
struct MATRIX x_thetadot; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #theta_dot_filt)
struct MATRIX Q_thetadot; ///< Covariance matrix of process noise of #theta_dot
struct MATRIX R_thetadot; ///< Covariance matrix of observation of #theta_dot
/** @} */

/**
 * @name Roll Kalman filtering matrices
 * These matrices pertain to the real-time Kalman filtering of the roll angle and angular rate
 * @{
 */
struct MATRIX P_phi; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #phi_filt estimate
struct MATRIX x_phi; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #phi_filt)
struct MATRIX Q_phi; ///< Covariance matrix of process noise of #phi
struct MATRIX R_phi; ///< Covariance matrix of observation of #phi
struct MATRIX P_phidot; ///< Predicted a priori and then updated a posteriori estimate covariance matrix of the #phi_dot_filt estimate
struct MATRIX x_phidot; ///< Predicted a priori and then updated a posteriori state estimate (the #MATRIX version of #phi_dot_filt)
struct MATRIX Q_phidot; ///< Covariance matrix of process noise of #phi_dot
struct MATRIX R_phidot; ///< Covariance matrix of observation of #phi_dot
/** @} */

struct MATRIX A_kalman; ///< Temporary matrix for Kalman filtering, dynamic equation x_dot=A_kalman*x ; y=C_kalman*x, but in discrete time!
struct MATRIX C_kalman; ///< Temporary matrix for Kalman filtering, dynamic equation x_dot=A_kalman*x ; y=C_kalman*x, but in discrete time!
struct MATRIX inn; ///< Temporary matrix "Innovation or measurement residual" (see <a href="http://en.wikipedia.org/wiki/Kalman_filter#Update">Wikipedia</a>)
struct MATRIX S; ///< Temporary matrix "Innovation (or residual) covariance" (see <a href="http://en.wikipedia.org/wiki/Kalman_filter#Update">Wikipedia</a>)
struct MATRIX K; ///< Temporary matrix "Optimal Kalman gain" (see <a href="http://en.wikipedia.org/wiki/Kalman_filter#Update">Wikipedia</a>)
struct MATRIX z_temp; ///< Temporary matrix used to convert the current scalar unfiltered signal value (e.g. #psi or #theta, etc.) into a [1x1] #MATRIX used in calculating #inn (see Kalman_filter()), needed due to how our linear algebra functions in la_header.h operate

//***************** Function declarations *******************
/** @cond INCLUDE_WITH_DOXYGEN */
void check_time(struct timeval *now, struct timeval before, struct timeval elapsed, unsigned long long int *time);
void write_to_file_custom(FILE *file_ptr, char *string,FILE *error_log);
void open_file(FILE **log, char *path, char *setting,FILE *error_log);
void open_error_file(FILE **error_log,char *path, char *setting);
void passive_wait(struct timeval *now,struct timeval *before,struct timeval *elapsed,unsigned long long int *time,unsigned long long int TIME__STEP);
void search_PWM(double thrust,unsigned char *pwm);
/** @endcond */
#endif /* MASTER_HEADER_H_ */
