/**
 * @file imu_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief IMU functions file (contains IMU comms, logs and filters).
 *
 * This file contains all necessary functions regarding the communication with the
 * IMU and the processing (filtering) of received data.
 */

# include <stdio.h>
# include <fcntl.h>
# include <unistd.h>
# include <errno.h>   // Error number definitions
# include <sys/termios.h> // UART serial communications library
# include <string.h>
# include <stdint.h>
# include <stdlib.h>
# include <sys/ioctl.h>
# include <sys/time.h>
# include <math.h>
# include <pthread.h>
# include "imu_header.h"
# include "master_header.h"
# include "la_header.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%% VARIABLE DEFINITIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%

unsigned char IMU_SYNCHED=0; ///< If =1, then the IMU and Raspberry Pi UART communication has been synced, =0 otherwise

unsigned char IMU_TX[2]="#f"; ///< Buffer holding transmit message to IMU (call to send Euler angles NOW)

/**
 * @name Average Euler angles group
 * Average Euler angle values obtained during calibration (zeroing) period
 */
/** @{ */
float psi_av=0; ///< Average value of psi (yaw) during calibration period
float theta_av=0; ///< Average value of theta (pitch) during calibration period
float phi_av=0; ///< Average value of phi (roll) during calibration period
/** @} */

/**
 * @name Last read Euler angles group
 * These are the last set of Euler angles that were read in from IMU during the previous iteration (1 time
 * step ago).
 */
/** @{ */
float psi_save_last=-9999.0; ///< Last read value of psi
float theta_save_last=-9999.0; ///< Last read value of theta
float phi_save_last=-9999.0; ///< Last read value of phi
/** @} */

int num_av_vars=0; ///< How many angles have we collected to average?

float dt; ///< The timestep for derivatives (time passed in [s] between current and last iteration)

//%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTION DEFINITIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%

/**
 * @fn void open_serial_port(int *fd,char *directory)
 *
 * This function opens the serial connection with the serial device connected to directory (e.g. /dev/ttyUSB0)
 * and saves the file handle to *fd.
 *
 * @param fd Pointer to the file handle.
 * @param directory Pointer to the string which defines the directory.
 */
void open_serial_port(int *fd,char *directory) {
	if ((*fd=open(directory, O_RDWR | O_NOCTTY | O_NONBLOCK))<0) {
		perror("CRITICAL ERROR: Failed to open the uart connection.\n");
		exit(-2); // Exit with a critical failure
	}
}

/**
 * @fn void close_port(int fd)
 *
 * This function closes the serial connection identified by handle fd.
 *
 * @param fd The file handle.
 */
void close_port(int fd) {
	if (close(fd)==-1) {
		perror("Error closed serial port!");
		exit(-2);
	}
}

/**
 * @fn void get_old_attr(int fd, struct termios *old_options)
 *
 * This function saves the current serial connection attributes old_attributes of the
 * connection identified by handle fd.
 *
 * @param fd The file handle.
 * @param old_options the old UART connection attributes.
 */
void get_old_attr(int fd, struct termios *old_options) {
	/* Description:
	 *
	 */
	if (tcgetattr(fd,old_options)!=0) {
		perror("Error getting old Razor IMU uart options!\n");
		close_port(fd);
		exit(-2);
	}
}

/**
 * @fn void reset_old_attr_port(int fd, struct termios *old_options)
 *
 * This function restores to the connection identified by handle fd its old settings, which
 * were saved back right after the connection was opened.
 *
 * @param fd The file handle.
 * @param old_options the old UART connection attributes.
 */
void reset_old_attr_port(int fd, struct termios *old_options) {
	/* Description:
	 *
	 */
	if (tcsetattr(fd,TCSANOW,old_options) != 0) {
		perror("Error restoring old options to serial port!\n");
		exit(-2);
	}
}

/**
 * @fn void set_new_attr(int fd, struct termios *old_options, struct termios *new_options)
 *
 * This function sets new_options for the connection identified by handle fd.
 *
 * @param fd The file handle.
 * @param old_options The old UART connection attributes.
 * @param new_options The new UART connection attributes that we want to use.
 */
void set_new_attr(int fd, struct termios *old_options, struct termios *new_options) {
	if (tcsetattr(fd,TCSANOW,new_options)!=0) {
		perror("Error setting the new Razor IMU uart options!\n");
		reset_old_attr_port(fd,old_options);
		close_port(fd);
		exit(-2);
	}
}

/**
 * @fn void set_to_blocking(int fd)
 *
 * We always open a serial connection with O_NONBLOCK in order to avoid the open() function hanging
 * forever due to a badly configured connection from before (not the fault of our program).
 * If we wish to use this connection with O_NONBLOCK disabled (i.e. in blocking mode) then we call this
 * function, which removes the O_NONBLOCK, thus setting the serial mode to <b>blocking</b>
 *
 * @param fd The file handle
 */
void set_to_blocking(int fd) {
	/* Description:
	 *
	 */
	int saved_args;
	if ((saved_args=fcntl(fd, F_GETFL))<0) {
		perror("fcntl F_GETFL failed for MSP430 UART.\n");
		exit(-2);
	}
	if ((fcntl(fd,F_SETFL,saved_args&~O_NONBLOCK))<0) { // Clear the non-blocking flag, i.e. set to blocking
		perror("fcntl F_GETFL failed for MSP430 UART.\n");
		exit(-2);
	}
}

/**
 * @fn float min_of_set(float now, float before)
 *
 * This function is used to overcome the difficulty of the atan2() function used in the Razor IMU
 * firmware wrapping in the +/-M_PI band, hence producing great discontinuities in the signal that would
 * make filtering useless. What we do is, given a value before, we return now2=now+x*2*M_PI where x is such
 * that the difference between now2 and before is minimized.
 *
 * Example : if before=+179 and now=-178 (because atan2() wrapped), then min_of_set(now,before) will return
 * now2=now+1*360=182 hence we don't have the wrapping problem anymore! Note that in the example degress were
 * used to facilitate understanding; the GNC program works in radians directly!
 *
 * @param now The value we want to adjust, which has possible wrapped and we want to "unwrap".
 * @param before The previously collected value, e.g. #psi_save_last, which has itself ALSO been adjusted in the previous iteration by min_of_set()!
 */
float min_of_set(float now, float before) {
	if (now<before) {
		temp2=now+2*M_PI-before;
		if ( temp2*temp2 < (now-before)*(now-before) ) {
			float ii=1;
			do {
				temp1=temp2;
				temp2=now+(ii+1)*2*M_PI-before;
				ii++;
			} while(temp1*temp1>temp2*temp2);
			return (temp1+before);
		} else {
			return now;
		}
	} else { // now>before
		temp2=now-2*M_PI-before;
		if ( temp2*temp2 < (now-before)*(now-before) ) {
			float ii=1;
			do {
				temp1=temp2;
				temp2=now-(ii+1)*2*M_PI-before;
				ii++;
			} while(temp1*temp1>temp2*temp2);
			return (temp1+before);
		} else {
			return now;
		}
	}
}

/**
 * @fn void Find_raw_Euler_angular_velocities()
 *
 * Take a numerical derivative of the euler angles to get angular rates [(rad)/s].
 */
void Find_raw_Euler_angular_velocities() {
	dt = (float)(time_imu)/1000000.0; // Convert the [us] timestep reading the IMU into [s]
	psi_dot=(psi_save-psi_save_last)/dt; // [rad/s] YAW RATE
	theta_dot=(theta_save-theta_save_last)/dt; // [rad/s] PITCH RATE
	phi_dot=(phi_save-phi_save_last)/dt; // [rad/s] ROLL RATE
}

/**
 * @fn void Treat_reply(char *comparison_string)
 *
 * This function handles user input, loops until the user inputs the right input (and gives cues to the
 * user if he/she doesn't input the right input).
 *
 * @param comparison_string The string that the user must enter.
 */
void Treat_reply(char *comparison_string) {
	do {
		scanf("%s",reply);
		if (strcmp(reply,comparison_string)!=0) {
			sprintf(ERROR_MESSAGE,"Wrong input! Type [%s]: ",comparison_string);
			printf("%s",ERROR_MESSAGE); fflush(stdout);
		}
	} while(strcmp(reply,comparison_string)); // Wait for user to type comparison_string
	memset(reply,0,sizeof(reply));
}

/**
 * @fn void construct_zeroed_DCM()
 *
 * Construct a DCM (Direct Cosine Matrix), i.e. a rotation from body (non-inertial)==>world (inertial) coordinates
 * based on currently stored yaw,pitch and roll matrices.
 *
 * Meaning of "zeroed" : we pre-multiply by R_MATRIX to get the DCM which is =[I] <==> the IMU is in the orientation at which it
 * was calibrated ("zeroed").
 */
void construct_zeroed_DCM() {
	DCM_MATRIX.matrix[0][0]=cos(theta_save)*cos(psi_save); // a1
	DCM_MATRIX.matrix[1][0]=cos(theta_save)*sin(psi_save); // a2
	DCM_MATRIX.matrix[2][0]=-sin(theta_save); // a3
	DCM_MATRIX.matrix[0][1]=sin(phi_save)*sin(theta_save)*cos(psi_save)-cos(phi_save)*sin(psi_save); // b1
	DCM_MATRIX.matrix[1][1]=sin(phi_save)*sin(theta_save)*sin(psi_save)+cos(phi_save)*cos(psi_save); // b2
	DCM_MATRIX.matrix[2][1]=sin(phi_save)*cos(theta_save); // b3
	DCM_MATRIX.matrix[0][2]=cos(phi_save)*sin(theta_save)*cos(psi_save)+sin(phi_save)*sin(psi_save); // c1
	DCM_MATRIX.matrix[1][2]=cos(phi_save)*sin(theta_save)*sin(psi_save)-sin(phi_save)*cos(psi_save); // c2
	DCM_MATRIX.matrix[2][2]=cos(phi_save)*cos(theta_save); // c3

	DCM_MATRIX=mmultiply(R_MATRIX,DCM_MATRIX); // Zero the DCM matrix with respect to the calibration orientation
}

/**
 * @fn float TO_DEG(float angle)
 *
 * Convert from radians to degrees
 */
float TO_DEG(float angle) {
	return angle*180.0/M_PI;
}

/**
 * @fn void zero_Euler_angles()
 *
 * Convert the Euler angles received from IMU and adjusted for wrapping by min_of_set() function into the
 * "zeroed" Euler angles in that they would all be =0.0 if the rocket is in the orientation at which it was
 * calibrated (i.e. at which its Euler angles were "zeroed"/at which the "zero point" of the Euler angles was
 * taken).
 */
void zero_Euler_angles() {
	theta_save = -asin(DCM_MATRIX.matrix[2][0]);
	psi_save = atan2(DCM_MATRIX.matrix[1][0],DCM_MATRIX.matrix[0][0]);
	phi_save = atan2(DCM_MATRIX.matrix[2][1],DCM_MATRIX.matrix[2][2]);
	if (psi_save_last!=-9999.0) { // Then we have history 1 time step back ==> can make sure angles don't wrap in [-180,180] degree range!
		// Make sure that filtered angles do not wrap
		psi_save=min_of_set(psi_save,psi_save_last);
		theta_save=min_of_set(theta_save,theta_save_last);
		phi_save=min_of_set(phi_save,phi_save_last);
	}
}

/**
 * @fn void Calibrate_IMU()
 *
 * Zero the IMU data, which means spend some time to get an average reading for psi, theta and phi and then use
 * that average reading to construct a matrix that would zero all three angles for the rocket orientation at which
 * the rocket is in when this function executes (i.e. rocket _s_t_a_t_i_o_n_n_a_r_y_ on launch pad).
 */
void Calibrate_IMU() {
	gettimeofday(&before_loop, NULL); // Get starting point for timing just before beginning the calibration
	gettimeofday(&before_imu, NULL);
	do {
		check_time(&now_loop,before_loop,elapsed_loop,&time_loop);

		passive_wait(&now_imu,&before_imu,&elapsed_imu,&time_imu,IMU__READ_TIMESTEP);

		// Register the values
		psi_save=psi;
		theta_save=theta;
		phi_save=phi;

		// Update averages
		psi_av=psi_av+psi_save;
		theta_av=theta_av+theta_save;
		phi_av=phi_av+phi_save;

		num_av_vars++;

		// Display values we are receiving
		printf("time_imu: %llu \t psi: %.4f \t theta: %.4f \t phi: %.4f\n",time_imu,TO_DEG(psi),TO_DEG(theta),TO_DEG(phi));
	} while(time_loop<=CALIB__TIME); // Calibrate while calibration time has not elapsed

	// Now do the average
	psi_av=psi_av/num_av_vars;
	theta_av=theta_av/num_av_vars;
	phi_av=phi_av/num_av_vars;

	// Now find the R_MATRIX which will annull the rotation matrix at the orientation at which the IMU was calibrated
	R_MATRIX.matrix[0][0]=cos(theta_av)*cos(psi_av); // a1
	R_MATRIX.matrix[0][1]=cos(theta_av)*sin(psi_av); // a2
	R_MATRIX.matrix[0][2]=-sin(theta_av); // a3
	R_MATRIX.matrix[1][0]=sin(phi_av)*sin(theta_av)*cos(psi_av)-cos(phi_av)*sin(psi_av); // b1
	R_MATRIX.matrix[1][1]=sin(phi_av)*sin(theta_av)*sin(psi_av)+cos(phi_av)*cos(psi_av); // b2
	R_MATRIX.matrix[1][2]=sin(phi_av)*cos(theta_av); // b3
	R_MATRIX.matrix[2][0]=cos(phi_av)*sin(theta_av)*cos(psi_av)+sin(phi_av)*sin(psi_av); // c1
	R_MATRIX.matrix[2][1]=cos(phi_av)*sin(theta_av)*sin(psi_av)-sin(phi_av)*cos(psi_av); // c2
	R_MATRIX.matrix[2][2]=cos(phi_av)*cos(theta_av); // c3
}

/**
 * @fn void Kalman_filter(struct MATRIX *x,struct MATRIX *P,float z,struct MATRIX Q,struct MATRIX R,float dt,struct MATRIX EYE2)
 *
 * This function applies a real-time Kalman filter on the signal z. Consequently, this function is called each time a new
 * value of z is read.
 *
 * @param x Predicted a priori and then updated a posteriori state estimate (it's the matrix version of the filtered value!).
 * @param P Predicted a priori and then updated a posteriori estimate covariance.
 * @param z The input, i.e. the noisy signal.
 * @param Q Covariance matrix of process noise (i.e. how much noise is there in the actual physics of the plant system?).
 * @param R Covariance matrix of observation (measurement) noise (i.e. how much noise is there is our sensors?).
 * @param dt The time step.
 * @param EYE2 A [2x2] identity matrix.
 */
void Kalman_filter(struct MATRIX *x,struct MATRIX *P,float z,struct MATRIX Q,struct MATRIX R,float dt,struct MATRIX EYE2) {
	// Initialize discrete-time model
	A_kalman.matrix[0][1]=dt;

	// Prediction
	*x = mmultiply(A_kalman,*x); // x=A*x
	*P = madd(mmultiply(A_kalman,mmultiply(*P,transpose(A_kalman))),Q); // P=A*P*A'+Q

	// Update
	z_temp.matrix[0][0]=z;
	inn = msubtract(z_temp,mmultiply(C_kalman,*x)); // inn=z-C*x
	S = madd(mmultiply(C_kalman,mmultiply(*P,transpose(C_kalman))),R); // S=C*P*C'+R
	K = mmultiply(*P,mmultiply(transpose(C_kalman),minverse_1x1(S))); // K=P*C'*inv(S)

	*x = madd(*x,mmultiply(K,inn)); // xNext=x+K*inn
	*P = mmultiply(msubtract(EYE2,mmultiply(K,C_kalman)),*P); // PNext=(eye(2)-K*C)*P
}

/**
 * @fn void *read_IMU_parallel(void *args)
 *
 * This is a (p)thread which does only one thing : read the raw IMU values:
 * 		- float psi
 * 		- float theta
 * 		- float phi
 * 		- float accelX
 * 		- float accelY
 * 		- float accelZ
 *
 * Once read, these values become available to be saved & used by other threads (such as the filtering thread).
 *
 * @param args A pointer to the input arguments (we have none for this thread)
 */
void *read_IMU_parallel(void *args) { // A thread for reading the IMU
	//######################### Now synch with the Razor IMU #########################
	if((write(RAZOR_UART,"#ob",3))<0) { // Turn on binary output
		perror("Failed to put Razor IMU into binary output mode (send \"#ob\").\n"); exit(-2);
	}
	if((write(RAZOR_UART,"#o1",3))<0) { // Turn on continuous streaming output
		perror("Failed to put Razor IMU into continuous streaming output mode (send \"#o1\").\n"); exit(-2);
	}
	if((write(RAZOR_UART,"#oe0",4))<0) { // Turn off error message output
		perror("Failed to put Razor IMU into no error message output mode (send \"#oe0\").\n"); exit(-2);
	}
	usleep(2000000);
	if ((tcflush(RAZOR_UART,TCIOFLUSH))==-1) { // Clear the input buffer up to here
		perror("Failed to flush the Razor IMU comm input buffer up to now.\n"); exit(-2);
	}
	if ((write(RAZOR_UART,"#s",2))<0) { // Request synch token!
		perror("Failed to request synch token from Razor IMU (send \"#s\").\n"); exit(-2);
	}

	//------- Find the synch token
	char SYNCH_TOKEN[2]="#S"; // The synch token we expect to receive from Razor IMU
	int token_matched=0; // =1 if token has been matched
	int iii;
	unsigned long int trial_counter=0;
	unsigned int global_trial_counter=0;
	while (!token_matched) { // Search for token
		token_matched=1; // Assume token has been found...
		for (iii=0;iii<2;iii++) {
			if (read(RAZOR_UART,IMU_SYNCH_RECEIVE,1)<0) { //; // Read in 1 character from buffer
				perror("Read failed.\n");
			}
			if (IMU_SYNCH_RECEIVE[0]!=SYNCH_TOKEN[iii]) {
				token_matched=0; //... by searching, proove that token has NOT been found unless this statement has not been reached, in which case
								 // token HAS been found!
				break;
			}
		}
		trial_counter++;
		if (trial_counter>=2000) { // If we have done "too many" trials trying to find synch token, then perhaps something went wrong and we missed it
								   // so try sending sanch signal again
			trial_counter=0;
			global_trial_counter++;
			if ((tcflush(RAZOR_UART,TCIOFLUSH))==-1) { // Clear the input buffer up to here
				perror("Failed to flush the Razor IMU comm input buffer up to now.\n"); exit(-2);
			}
			if ((write(RAZOR_UART,"#s",2))<0) { // Request synch token!
				perror("Failed to request synch token from Razor IMU (send \"#s\").\n"); exit(-2);
			}
		}
		if (global_trial_counter>=10) {
			printf("Failed to synch with Razor IMU (global_trial_counter=%u). Quitting.\n",global_trial_counter);
			exit(-2); // Exit with a failure
		}
	}

	// Change blocking read parameters for IMU to the 24 bytes we expect to receive each time
	new_razor_uart_options.c_cc[VTIME] = 0; // Return characters over UART immediately
	new_razor_uart_options.c_cc[VMIN] = MAX_BUFFER; // Return once MAX_BUFFER characters have been received over the UART
	set_new_attr(RAZOR_UART,TCSANOW,&new_razor_uart_options); // Set the new options for the port...
	//######################### Finish synch with the Razor IMU #########################
	IMU_SYNCHED=1; // Notify others that synchronization has been done
	//######################### An infinite loop now (until cancelled) for reading the raw IMU data
	do {
		if ((read(RAZOR_UART,IMU_RX,MAX_BUFFER))<0) { // Instruction waits for 24 bytes to be received over UART from Razor IMU
			perror("Unable to read from Razor IMU UART.\n");
			exit(-2); // Exit with failure
		}

		/* Convert the recorded 24-bytes array sent by the IMU to 6 floating point values : the yaw, pitch, roll and 3 accelerations (along X,Y,Z).
		 * Note that even though, as in the three line-comments below, the angle bytes are written by blocks of 4
		 * with MSB b_Y_te _L_A_S_T_ and LSB b_Y_te _F_I_R_S_T_ by the Arduino onboard the Razor IMU, the endianness of the Linux platform makes direct memcpy work even
		 * though the array written into yaw_bytes, as for the other *_bytes, has LSB at [0] location and MSB at [3] location
		 * (Raspberry pi used is apparently little-endian)
		 */
		// Yaw={IMU_RX[3],IMU_RX[2],IMU_RX[1],IMU_RX[0]}
		// Pitch={IMU_RX[7],IMU_RX[6],IMU_RX[5],IMU_RX[4]}
		// Roll={IMU_RX[11],IMU_RX[10],IMU_RX[9],IMU_RX[8]}
		memcpy(&yaw_bytes,&IMU_RX[0],4*sizeof(unsigned char));
		memcpy(&pitch_bytes,&IMU_RX[4],4*sizeof(unsigned char));
		memcpy(&roll_bytes,&IMU_RX[8],4*sizeof(unsigned char));
		memcpy(&accelX_bytes,&IMU_RX[12],4*sizeof(unsigned char));
		memcpy(&accelY_bytes,&IMU_RX[16],4*sizeof(unsigned char));
		memcpy(&accelZ_bytes,&IMU_RX[20],4*sizeof(unsigned char));
		// Typecast the 4 byte slices into IEEE754 float values
		psi = *((float*)yaw_bytes);
		theta = *((float*)pitch_bytes);
		phi = *((float*)roll_bytes);
		accelX = *((float*)accelX_bytes);
		accelY = *((float*)accelY_bytes);
		accelZ = *((float*)accelZ_bytes);
	} while(!IMU_quit); // Continue reading sensor until quit

	printf("\nQuitting IMU reading thread!\n");
	pthread_exit(NULL); // Quit the pthread
}

/**
 * @fn void *get_filtered_attitude_parallel(void *args)
 *
 * This (p)thread does the sole job of filtering received data from the IMU. It is cadenced at the 1/IMU__READ_TIMESTEP [MHz] frequency
 * and so, each time that an interation is done, it collects the most recently available data from the IMU (stored in psi,theta,phi,accelX,
 * accelY and accelZ variables) and processes/filters it, then saves it to a log file.
 *
 * @param args A pointer to the input arguments (we have none for this thread)
 */
void *get_filtered_attitude_parallel(void *args) { // A thread for reading the IMU
	char IMU_MESSAGE[200];
	write_to_file_custom(imu_log,"time_imu_glob \t dt \t psi_save \t theta_save \t phi_save \t psi_dot \t theta_dot \t phi_dot \t psi_filt \t theta_filt \t phi_filt \t psi_dot_filt \t theta_dot_filt \t phi_dot_filt \t wx \t wy \t wz \t accelX_save \t accelY_save \t accelZ_save\n",error_log);

	gettimeofday(&before_imu, NULL); // Get initial read time
	do {
		check_time(&now_imu_glob,GLOBAL__TIME_STARTPOINT,elapsed_imu_glob,&time_imu_glob);

		passive_wait(&now_imu,&before_imu,&elapsed_imu,&time_imu,IMU__READ_TIMESTEP); // Control execution frequency of the loop

		// Register angles
		psi_save=psi;
		theta_save=theta;
		phi_save=phi;
		accelX_save=accelX;
		accelY_save=accelY;
		accelZ_save=accelZ;

		// Zero out the angles
		construct_zeroed_DCM();
		zero_Euler_angles();
		Find_raw_Euler_angular_velocities();
		psi_save_last=psi_save; theta_save_last=theta_save; phi_save_last=phi_save; // Memorize the angles for next iteration


		// Now that raw values have been read in, it is time to apply kalman filtering
		Kalman_filter(&x_psi,&P_psi,psi_save,Q_psi,R_psi,dt,EYE2);
		Kalman_filter(&x_psidot,&P_psidot,psi_dot,Q_psidot,R_psidot,dt,EYE2);
		Kalman_filter(&x_theta,&P_theta,theta_save,Q_theta,R_theta,dt,EYE2);
		Kalman_filter(&x_thetadot,&P_thetadot,theta_dot,Q_thetadot,R_thetadot,dt,EYE2);
		Kalman_filter(&x_phi,&P_phi,phi_save,Q_phi,R_phi,dt,EYE2);
		Kalman_filter(&x_phidot,&P_phidot,phi_dot,Q_phidot,R_phidot,dt,EYE2);
		psi_filt=x_psi.matrix[0][0];
		psi_dot_filt=x_psidot.matrix[0][0];
		theta_filt=x_theta.matrix[0][0];
		theta_dot_filt=x_thetadot.matrix[0][0];
		phi_filt=x_phi.matrix[0][0];
		phi_dot_filt=x_phidot.matrix[0][0];
		wx=phi_dot_filt-psi_dot_filt*sin(theta_filt);
		wy=theta_dot_filt*cos(phi_filt)+psi_dot_filt*cos(theta_filt)*sin(phi_filt);
		wz=psi_dot_filt*cos(theta_filt)*cos(phi_filt)-theta_dot_filt*sin(phi_filt);

		sprintf(IMU_MESSAGE,"%llu\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n",time_imu_glob,dt,psi_save,theta_save,phi_save,psi_dot,theta_dot,phi_dot,psi_filt,theta_filt,phi_filt,psi_dot_filt,theta_dot_filt,phi_dot_filt,wx,wy,wz,accelX_save,accelY_save,accelZ_save);
		write_to_file_custom(imu_log,IMU_MESSAGE,error_log);
	} while(!IMU_quit); // Continue reading sensor until quit

	printf("\nQuitting filtering thread!\n");
	pthread_exit(NULL); // Quit the pthread
}
