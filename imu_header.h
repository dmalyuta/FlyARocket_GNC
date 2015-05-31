/**
 * @file imu_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief IMU header file.
 *
 * This is the header to imu_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef IMU_HEADER_H_
#define IMU_HEADER_H_

# include <termios.h>

# define MAX_BUFFER 24 ///< The max buffer size for receving data from IMU

unsigned char IMU_RX[MAX_BUFFER]; ///< Buffer holding received values via UART from Razor IMU
char IMU_SYNCH_RECEIVE[1]; ///< Buffer used for receving the 2-character (2-byte) synch token from the IMU during sychronization.
extern unsigned char IMU_TX[2]; ///< Buffer holding transmit message to IMU (call to send Euler angles NOW)
int RAZOR_UART; ///< Holds Razor IMU connection file

struct termios new_razor_uart_options; ///< New IMU UART connection options
struct termios old_razor_uart_options; ///< Old IMU UART connection options (those that were initially present when program started)

extern unsigned long int CALIB__TIME; ///< Calibration time [us], i.e. microseconds

/**
 * @name IMU reception group
 * variables holdin the raw values received from IMU
 */
/** @{ */
float psi; ///< Yaw angle
unsigned char yaw_bytes[4]; ///< Yaw angle bytes used for conversion to float
float theta; ///< Pitch angle
unsigned char pitch_bytes[4]; ///< Pitch angle bytes used for conversion to float
float phi; ///< Roll angle
unsigned char roll_bytes[4]; ///< Roll angle bytes used for conversion to float
float accelX; ///< X-acceleration
unsigned char accelX_bytes[4]; ///< X-acceleration angle bytes used for conversion to float
float accelY; ///< Y-acceleration
unsigned char accelY_bytes[4]; ///< Y-acceleration angle bytes used for conversion to float
float accelZ; ///< Z-acceleration
unsigned char accelZ_bytes[4]; ///< Z-acceleration angle bytes used for conversion to float
/** @} */

/**
 * @name Current accelerations
 * These are the accelerations saved into the filtering thread get_filtered_attitude_parallel()
 * when it does an iteration, hence they are equal to the most recent #accelX, #accelY and #accelZ that
 * have been read from the IMU. The filtering thread simply logs them into the #imu_log file.
 */
/** @{ */
float accelX_save; ///< Saved X-acceleration
float accelY_save; ///< Saved Y-acceleration
float accelZ_save; ///< Saved Z-acceleration
/** @} */

/**
 * @name Current Euler angles group
 * These angles are the ones saved into the filtering thread get_filtered_attitude_parallel()
 * when it does an iteration, hence they are equal to the most recent #psi, #theta and #phi that
 * have been read from the IMU.
 */
/** @{ */
float psi_save; ///< Saved yaw angle
float theta_save; ///< Saved pitch angle
float phi_save; ///< Saved roll angle
/** @} */

/**
 * @name Last read Euler angles group
 * These are the last set of Euler angles that were read in from IMU during the previous iteration (1 time
 * step ago).
 */
/** @{ */
extern float psi_save_last;
extern float theta_save_last;
extern float phi_save_last;
/** @} */

/**
 * @name Filtered Euler angles and angular rates
 * These are the filtered versions of the #psi_save, #theta_save and #phi_save signals and their numerical
 * derivatives (see Find_raw_Euler_angular_velocities()).
 */
/** @{ */
float psi_filt; ///< Filtered yaw
float psi_dot_filt; ///< Filtered yaw rate
float theta_filt; ///< Filtered pitch
float theta_dot_filt; ///< Filtered pitch rate
float phi_filt; ///< Filtered roll
float phi_dot_filt; ///< Filtered roll rate
/** @} */

/**
 * @name Average Euler angles group
 * Average Euler angle values obtained during calibration (zeroing) period
 */
/** @{ */
extern float psi_av;
extern float theta_av;
extern float phi_av;
/** @} */

extern int num_av_vars;

/**
 * @name Euler angular rates group
 * The unfiltered, noisy numerical time derivatives of the Euler angles read in from the IMU
 */
/** @{ */
float psi_dot; ///< Time derivative of psi
float theta_dot; ///< Time derivative of theta
float phi_dot; ///< Time derivative of phi
/** @} */

/**
 * @name Body rates group
 * These are the BODY rates (angular velocity about X Y and Z body axes of rocket)
 */
/** @{ */
float wx; ///< X-body rate
float wy; ///< Y-body rate
float wz; ///< Z-body rate
/** @} */

char reply[30]; ///< User input string

float temp1; ///< Temp variable used in min_of_set()
float temp2; ///< Temp variable used in min_of_set()

struct MATRIX R_MATRIX; ///< Matrix which zeroes the Euler angles for the calibrated orientation
struct MATRIX DCM_MATRIX; ///< Direct Cosine Matrix

extern float dt;

struct MATRIX EYE2; ///< [2x2] identity matrix, used in Kalman_filter()

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTION DECLARATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/** @cond INCLUDE_WITH_DOXYGEN */
void close_port(int fd);
void open_serial_port(int *fd,char *directory);
void get_old_attr(int fd, struct termios *old_options);
void set_to_blocking(int fd);
void set_new_attr(int fd, struct termios *old_options, struct termios *new_options);
void reset_old_attr_port(int fd, struct termios *old_options);
float min_of_set(float now, float before);
void Treat_reply(char *comparison_string);
void construct_zeroed_DCM();
void Find_raw_Euler_angular_velocities();
float TO_DEG(float angle);
void zero_Euler_angles();
void Calibrate_IMU();
void Kalman_filter(struct MATRIX *x,struct MATRIX *P,float z,struct MATRIX Q,struct MATRIX R,float dt,struct MATRIX EYE2);
void *read_IMU_parallel(void *args);
void *get_filtered_attitude_parallel(void *args);
/** @endcond */

#endif /* IMU_HEADER_H_ */
