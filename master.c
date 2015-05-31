/**
 * @file master.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Master GNC source file (main function file).
 *
 * This is the master file of the GNC program which manages all parallel threads
 * and calls all necessary functions for the collection and processing of data
 * as well as for rocket control.
 */

# include <stdio.h>
# include <fcntl.h>
# include <unistd.h>
# include <stdint.h>
# include <stdlib.h>
# include <termios.h> // For UART serial communication
# include <linux/spi/spidev.h> // For SPI communication
# include <signal.h> // Catch Ctrl-C
# include <sys/time.h> // For the (gettimeofday) function used to get millisecond time
# include <math.h> // For sin(), cos(), etc. functions
# include <string.h> // For string functions like (strlen)
# include <pthread.h> // Multi-threading (code parallelization)

# include "control_header.h"
# include "master_header.h"
# include "imu_header.h"
# include "la_header.h"
# include "msp430_header.h"
# include "simplex_header.h"
# include "rpi_gpio_header.h"
# include "spycam_header.h"
# include "pressure_header.h"


// *********************************************************************
// **************************** ASSIGN GLOBALS *************************
// *********************************************************************

unsigned long long int ENGINE__BURN_TIME=1100000; ///< Upper bountd on time [us] between engine start and engine burnout // TODO : change this with Xavier!
unsigned long long int ACTIVE__CONTROL_TIME=20000000; ///< Time [us] during which control loop will be active // TODO : change this with Xavier!
unsigned long long int DESCENT__TIME=15000000; ///< Time [us] for rocket descent with parachute (i.e. between parachutes opening and a soft touchdown) // TODO : change this with Xavier!
unsigned long long int CONTROL__TIME_STEP=20000; ///< =1/(control loop frequency [MHz]), the time interval between applying control, in [us]
unsigned long long int SPI__READ_TIMESTEP=20000; // Timestep [us] at which pressure/temperature is read from SPI sensor
unsigned long long int IMU__READ_TIMESTEP=20000; // Timestep [us] at which filtered rocket attitude is obtained
unsigned long int CALIB__TIME = 5000000; // 5000000 [us]==5 [second] calibration time

unsigned char SPI_quit=0; // By default don't quit reading the pressure sensor!
unsigned char IMU_quit=0; // By default don't quit reading the pressure sensor!

unsigned char PWM1=0; // PWM value for the R1 valve
unsigned char PWM2=0; // PWM value for the R2 valve
unsigned char PWM3=0; // PWM value for the R3 valve
unsigned char PWM4=0; // PWM value for the R4 valve
double R1=0; // Valve R1 thrust
double R2=0; // Valve R2 thrust
double R3=0; // Valve R3 thrust
double R4=0; // Valve R4 thrust

double d=0.005; // [m] offset distance of RCS valves from centerline (for roll control) // TODO: CHECK THIS WITH GAUTIER
double Fpitch=0; // Pitch force (parallel to body -Z axis, so as to produce positive pitch rate when Fpitch>0 (right hand rule))
double Fyaw=0; // Yaw force (parallel to body +Y axis, so as to produce positive yaw rate when Fyaw>0 (right hand rule))
double Mroll=0; // Roll moment (positive about +X axis, so as to produce positive roll rate when Mroll>0 (right hand rule))

int N=4; // Number of variables in cost function ==> R1, R2, R3, R4 so 4 variables
int M1=0; // No (<=) type inequality constraints
int M2=0; // No (>=) type inequality constraints
int M3=3; // 3 (=) type constraints (for Fpitch, Fyaw, Mroll)
int M=3; // Total number of constraints (M=M1+M2+M3)

/**
 * @name Control algorithm input variables
 * Below 6 variables are the ones that the control algorithm "sees", as in that they are updated at our CONTROL frequence,
 * which we decide, while the angles read from the IMU are read as fast as possible to data-log all information on rocket
 * orientation!
 * @{
 */
float psi_cont=0; ///< Yaw angle
float psidot_cont=0; ///< Yaw rate
float theta_cont=0; ///< Pitch angle
float thetadot_cont=0; ///< Pitch rate
float phi_cont=0; ///< Roll angle
float wx_cont=0; ///< X-body rate
/** @} */

/**
 * @name Reference angles and rates
 * Below are the values that we want to achieve when controlling the rocket. In our case they are 0 because we want to achieve a
 * steay, perfectly vertical orientation. But they can be even time-varying in other applications - e.g. for guiding the rocket along
 * a trajectory.
 * @{
 */
float psi_ref=0; ///< Yaw angle reference [(rad)]
float theta_ref=0; ///< Pitch angle reference [(rad)]
float wx_ref=0; ///< Roll rate reference [(rad)/s]
/** @} */

char flight_type=0; ///< =1 for active control flight, =0 for passive flight (i.e. only data logging)

FILE *error_log=NULL;
FILE *pressure_log=NULL;
FILE *imu_log=NULL;
FILE *control_log=NULL;

struct bcm2835_peripheral gpio = {GPIO_BASE}; ///< Our access register to the Raspberry Pi's GPIOs
unsigned char launch_detect_gpio=12; ///< Number of GPIO (i.e. GPIO<num>) to which the launch umbillical cable is connected and hence which detects the launch

// *********************************************************************
// ***************************** MAIN FUNCTION *************************
// *********************************************************************
/**
 * @fn int main(void)
 * <b>This is the main function of the flight software</b>. This function is what defines
 * the sequence of steps that occur during pre-flight and flight and what orchestrates all
 * processes that occur, such as the starting of parallel threads, of active control and of
 * data logging. It accepts no input parameters.
 */
int main(void) {
	gettimeofday(&GLOBAL__TIME_STARTPOINT, NULL); // Get starting point for timing just before beginning the calibration

	//############################ DATA LOGGING SETUP START ############################
	if (pthread_mutex_init(&error_log_write_lock, NULL) != 0) { // Initialize the mutex lock protecting from writing into error file simultaneously by more than 1 thread
		perror("Failed to initialize error log write mutex.");
		exit(-2);
	}

	printf("Opening log files... ");

	open_error_file(&error_log,"./logs/error_log.txt","w");
	open_file(&pressure_log,"./logs/pressure_log.txt","w",error_log);
	open_file(&imu_log,"./logs/imu_log.txt","w",error_log);
	open_file(&control_log,"./logs/control_log.txt","w",error_log);

	printf("opened.\n");
	//############################ DATA LOGGING SETUP END ##############################

	//############################ CAMERA RECORDING SETUP START ##############################
	stopVideo();
	printf("Starting spy camera recording... ");
	startVideo("flight_recording.h264", ""); // TODO : set appropriate camera options with Raimondo
	usleep(1000000); // Sleep for 1 second while camera is started
	printf("started. \n");
	//############################ CAMERA RECORDING SETUP END ################################

	//############################ GPIO SETUP START #################################
	if(map_peripheral(&gpio) == -1) {
		printf("Failed to map the physical GPIO registers into the virtual memory space.\n");
		stopVideo();
		exit(-2); // Exit with a critical failure
	}
	INP_GPIO(launch_detect_gpio); // Set pin 32 (GPIO12 on the Raspberry Pi Model B+) as an input
	//############################ GPIO SETUP END #################################

	//############################ PRESSURE SENSOR SETUP START #################################
	// Here we open the SPI connection to the Honeywell HSC (differential) pressure sensors
	// Radial sensor captures pressure on side of nose cone
	// Axial sensor capture pressure right at tip of nose cone, looking into the head wind

	SPI_config.mode=0;
	SPI_config.bits=8;
	SPI_config.max_speed=800000;
	SPI_config.buffer_length=BYTE_NUMBER;

	SPI_config.P_OUT__MAX=14745;
	SPI_config.P_OUT__MIN=1638;
	SPI_config.P__MAX=100;
	SPI_config.P__MIN=-100;

	SPI_config.radial_sensor_fd=0;
	SPI_config.axial_sensor_fd=0;

	int ii;
	for (ii=0;ii<SPI_config.buffer_length;ii++) {
		data[ii]=0; // Reset the value in data buffer to zero ==> send whatever (MOSI line not connected to sensor anyway), receive back the sensor reading

		memset(transfer+ii,0,sizeof(struct spi_ioc_transfer)); // Reset transfer struct to NULL, otherwise does not work!
		transfer[ii].tx_buf = (unsigned long)(data+ii);	// Buffer for SENDING data (MOSI)
		transfer[ii].rx_buf = (unsigned long)(data+ii); //  Buffer for RECEIVING data (MISO)
		transfer[ii].len = sizeof(*(data+i)); // Length of buffer, i.e. size in bytes of data[i] (NB: data[i]==*(data+i) in pointer arithmetic)
		transfer[ii].speed_hz = SPI_config.max_speed; // Speed in [Hz]
		transfer[ii].bits_per_word = SPI_config.bits; // Bits per transmission ("per word")
		transfer[ii].delay_usecs = 100; // Delay in [us]
		transfer[ii].cs_change = 0;
	}

	printf("Connecting to Honeywell sensors... ");

	pressure_sensor_SPI_connect(RADIAL_SENSOR,&(SPI_config.radial_sensor_fd),SPI_config.mode,SPI_config.bits,SPI_config.max_speed); // Open SPI connection to the radial pressure/temperature sensor
	pressure_sensor_SPI_connect(AXIAL_SENSOR,&(SPI_config.axial_sensor_fd),SPI_config.mode,SPI_config.bits,SPI_config.max_speed); // Open SPI connection to the axial pressure/temperature sensor

	printf("connected.\n");

	sprintf(MESSAGE,"~~~~~~~~Sensor SPI info~~~~~~~~\nSPI Mode is: %u\nSPI bits is: %u\nSPI speed is: %lu [Hz]\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n",SPI_config.mode,SPI_config.bits,SPI_config.max_speed);
	printf(MESSAGE);

	// Now temporarily read sensor values to make sure that everything is OK with sensor readings before flight
	printf("Type [TEST] to view pressure sensor readings: ");
	Treat_reply("TEST");

	pthread_t SPI_pressure_thread;

	if (pthread_create(&SPI_pressure_thread,NULL,get_readings_SPI_parallel,(void *) &SPI_config)) { // (void *) &SPI_config means cast a pointer to a (struct SPI_data) to a pointer to a (void), which is the only thing a pthread can accept
		perror("Failed to create SPI pressure sensor thread.");
		stopVideo();
		exit(-2);
	}

	gettimeofday(&before_loop, NULL); // Get starting point for timing just before beginning the calibration
	gettimeofday(&before_control, NULL);
	do {
		check_time(&now_loop,before_loop,elapsed_loop,&time_loop);

		passive_wait(&now_control,&before_control,&elapsed_control,&time_control,SPI__READ_TIMESTEP);

		printf("radial_status: %c \t radial p: %.4f \t radial T: %.4f \t axial_status: %c \t axial p: %.4f \t axial T: %.4f\n",radial_status,radial_pressure,radial_temperature,axial_status,axial_pressure,axial_temperature);
	} while(time_loop<=CALIB__TIME); // Show pressure values until CALIB__TIME elapses (don't define separate time variable for conciseness)

	printf("\nIs this OK? Type [Calibrate] to continue: ");
	Treat_reply("Calibrate");
	//############################ PRESSURE SENSOR SETUP END #################################

	//############################ CONTROL SETUP START ############################
	printf("Setting up control coefficients... ");

	// The below coefficients were developed through MATLAB/Simulink control loop design. They are hard-coded here.
	Fpitch_loop_control_setup();
	Fyaw_loop_control_setup();
	Mroll_loop_control_setup();

	printf("setup.\n");
	//############################ CONTROL SETUP END ##############################

	//############################ RAZOR IMU SETUP START ############################
	// Many thanks to user @tchar on Stackoverflow at http://stackoverflow.com/questions/21411385/having-problems-with-serial-port-on-linux-between-avr-and-linux/30490312#30490312
	// For this solution to how to properly connect over serial UART between Linux and an avr microcontroller (AtMega328 on the IMU here)
	// Maybe the steps below also work for UART connections to other devices
	// Main idea : order step 1. ---> step 4. _M_A_T_T_E_R_S_ !!!
	// 1. Define UART options that we want for the Razor IMU
	memset(&new_razor_uart_options,0,sizeof(new_razor_uart_options));
	new_razor_uart_options.c_iflag = 0;
	new_razor_uart_options.c_oflag = 0;
	new_razor_uart_options.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	new_razor_uart_options.c_lflag = 0;
	new_razor_uart_options.c_cc[VMIN] = 0;
	new_razor_uart_options.c_cc[VTIME] = 1;
	// 2. Open the Razor IMU serial port
	printf("Opening Razor IMU UART connection... ");
	open_serial_port(&RAZOR_UART,"/dev/ttyUSB0");
	printf("opened.\n"); fflush(stdout);
	// 3. Get the existing Razor IMU uart options
	get_old_attr(RAZOR_UART,&old_razor_uart_options);
	// 4. Set new uart options
	// First set to blocking mode
	set_to_blocking(RAZOR_UART);
	set_new_attr(RAZOR_UART,&old_razor_uart_options,&new_razor_uart_options);

	R_MATRIX=initMatrix(3,3);
	DCM_MATRIX=initMatrix(3,3);

	// Begin IMU reading thread
	pthread_t IMU_thread;
	if (pthread_create(&IMU_thread,NULL,read_IMU_parallel,NULL)) {
		perror("Failed to create IMU reading thread.");
		stopVideo();
		exit(-2);
	}
	// From now on, raw IMU data is available for access from all threads!

	while (!IMU_SYNCHED) {} // Wait for sync from IMU
	usleep(IMU__READ_TIMESTEP); // Wait to be sure that now reading IMU data properly (not 0,0,0 angles...)

	Calibrate_IMU(); // Calibrate IMU
	construct_zeroed_DCM();
	zero_Euler_angles();
	psi_save_last=psi_save; theta_save_last=theta_save; phi_save_last=phi_save;

	printf("\n\nFinished calibrating. The zeroed angles are now:\n\n");
	sprintf(MESSAGE,"Yaw (psi) = %.4f\nPitch (theta) = %.4f\nRoll (phi) = %.4f\n\n",TO_DEG(psi_save),TO_DEG(theta_save),TO_DEG(phi_save));
	printf("%s",MESSAGE);
	printf("Is this OK? Type [Filter] to continue: ");
	Treat_reply("Filter");
	//############################ RAZOR IMU SETUP END ##############################

	//############################ SIGNAL FILTERING SETUP START ############################
	// Here we begin filtering the IMU euler angles and rates using the Kalman filter
	/////////////////////////////// PSI FILTER SETUP ///////////////////////////////
	P_psi=initMatrix(2,2); // Initial covariance matrix of the psi estimate
	P_psidot=initMatrix(2,2);
	x_psi=initMatrix(2,1);
	x_psidot=initMatrix(2,1);
	Q_psi=initMatrix(2,2); // State error covariance matrix for psi,psi_dot
	Q_psidot=initMatrix(2,2);
	R_psi=initMatrix(1,1); // Output covariance matrix for psi,psi_dot
	R_psidot=initMatrix(1,1);

	P_psi.matrix[0][0] = 1;		P_psi.matrix[0][1] = 0;
	P_psi.matrix[0][1] = 0;		P_psi.matrix[1][1] = 1;

	P_psidot.matrix[0][0] = 1;		P_psidot.matrix[0][1] = 0;
	P_psidot.matrix[0][1] = 0;		P_psidot.matrix[1][1] = 1;

	x_psi.matrix[0][0] = 0;
	x_psi.matrix[1][0] = 0;

	x_psidot.matrix[0][0] = 0;
	x_psidot.matrix[1][0] = 0;

	Q_psi.matrix[0][0] = 0.01;	Q_psi.matrix[0][1] = 0;
	Q_psi.matrix[0][1] = 0;		Q_psi.matrix[1][1] = 100;

	Q_psidot.matrix[0][0] = 200;	Q_psidot.matrix[0][1] = 0;
	Q_psidot.matrix[0][1] = 0;		Q_psidot.matrix[1][1] = 200;

	R_psi.matrix[0][0] = 10;
	R_psidot.matrix[0][0] = 5000;
	// We filter each signal the same way so matrices below are initialized to the same values as for psi
	/////////////////////////////// THETA FILTER SETUP ///////////////////////////////
	P_theta=P_psi;
	x_theta=x_psi;
	Q_theta=Q_psi;
	R_theta=R_psi;

	P_thetadot=P_psidot;
	x_thetadot=x_psidot;
	Q_thetadot=Q_psidot;
	R_thetadot=R_psidot;
	/////////////////////////////// PHI FILTER SETUP ///////////////////////////////
	P_phi=P_psi;
	x_phi=x_psi;
	Q_phi=Q_psi;
	R_phi=R_psi;

	P_phidot=P_psidot;
	x_phidot=x_psidot;
	Q_phidot=Q_psidot;
	R_phidot=R_psidot;

	// Define the [2x2] identity matrix
	EYE2=initMatrix(2,2);
	EYE2.matrix[0][0]=1;	EYE2.matrix[0][1]=0;
	EYE2.matrix[1][0]=0;	EYE2.matrix[1][1]=1;

	//--------------- Allocate space for other matrices used in Kalman filtering -----------
	A_kalman=initMatrix(2,2);
	C_kalman=initMatrix(1,2);
	z_temp = initMatrix(1,1);
	inn = initMatrix(1,1);
	S = initMatrix(1,1);
	K = initMatrix(2,1);

	A_kalman.matrix[0][0]=1;
	A_kalman.matrix[1][0]=0;
	A_kalman.matrix[1][1]=1;

	C_kalman.matrix[0][0]=1;	C_kalman.matrix[0][1]=0;
	//---------------------------------------------------------------------------------------

	// Now spend 5 seconds filtering the signals
	// Then we are sure that {psi,psi_dot,theta,theta_dot,phi,phi_dot} signals are well filtered and all *_last variables are
	// available such that we can ready ourselves for passing into the main control loop upon launch detection
	printf("Beginning Kalman filtering in 1 second.\n"); fflush(stdout);
	usleep(1000000);

	//----------- Create filtering thread
	pthread_t Filt_thread;
	if (pthread_create(&Filt_thread,NULL,get_filtered_attitude_parallel,NULL)) {
		perror("Failed to create filtering thread.");
		stopVideo();
		exit(-2);
	}

	gettimeofday(&before_loop, NULL);
	gettimeofday(&before_control, NULL);
	do {
		check_time(&now_loop,before_loop,elapsed_loop,&time_loop);

		passive_wait(&now_control,&before_control,&elapsed_control,&time_control,IMU__READ_TIMESTEP);

		printf("dt: %.4f \t psi_filt: %.2f \t psi_dot_filt: %.2f \t theta_filt: %.2f \t theta_dot_filt: %.2f \t phi_filt: %.2f \t phi_dot_filt: %.2f\n",dt,psi_filt,psi_dot_filt,theta_filt,theta_dot_filt,phi_filt,phi_dot_filt);
	} while(time_loop<=CALIB__TIME);

	printf("\n\nFinished filtering.\n");
	printf("Is this OK? Type [Continue] to continue: ");
	Treat_reply("Continue");
	//############################ SIGNAL FILTERING SETUP END ############################

	//############################ WAIT FOR LAUNCH ############################
	printf("Is this a controlled (active) or uncontrolled (passive) flight? Type [ACTIVE] or [PASSIVE]: ");
	char not_done=1;
	do {
		scanf("%s",reply);
		if ((strcmp(reply,"ACTIVE")!=0) && (strcmp(reply,"PASSIVE")!=0)) {
			printf("Wrong input! Type [ACTIVE] or [PASSIVE]: "); fflush(stdout);
		} else if (strcmp(reply,"ACTIVE")==0) {
			flight_type=1; not_done=0;

			//############################ MSP430 SETUP START ############################
			// 1. Define UART options that we want for the Razor IMU
			memset(&new_msp430_uart_options,0,sizeof(new_msp430_uart_options));
			new_msp430_uart_options.c_iflag = 0;
			new_msp430_uart_options.c_oflag = 0;
			new_msp430_uart_options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
			new_msp430_uart_options.c_lflag = 0;
			new_msp430_uart_options.c_cc[VMIN] = 0;
			new_msp430_uart_options.c_cc[VTIME] = 1;
			// 2. Open the Razor IMU serial port
			printf("Opening MSP430 UART connection... "); fflush(stdout);
			open_serial_port(&MSP430_UART,"/dev/ttyAMA0");
			printf("opened.\n");
			// 3. Get the existing Razor IMU uart options
			get_old_attr(MSP430_UART,&old_msp430_uart_options);
			// 4. Set new uart options
			// First set to blocking mode
			set_to_blocking(MSP430_UART);
			set_new_attr(MSP430_UART,&old_msp430_uart_options,&new_msp430_uart_options);

			// Just in case MSP430 has not reset (is not at start of program), we attempt to reset it even before saying "hi"
			printf("Resetting MSP430G2553 microcontroller..."); fflush(stdout);
			MSP430_UART_write("@e!");
			usleep(500000); // Wait to make sure MSP430 has had time to back to start of program and begin waiting for a start handshake ("@s!")
			printf(" reset.\n");

			// Now start the MSP430
			printf("Saying Hi to MSP430G2553 slave microcontroller (sending \"@s!\")... "); fflush(stdout);
			MSP430_UART_write("@s!"); // Write the agreed-upon message (@s!) which the MSP430 code understands as
									  // "Raspberry Pi master is telling me to turn on my interrupts, play my warning message and enter my main while(1) loop"
			printf("sent.\n");
			usleep(10000000); // Wait 10 seconds while MSP430 plays the warning sound that it has been activated (that "@s!" start program instruction has been received)
			//############################ MSP430 SETUP END ############################

		} else if (strcmp(reply,"PASSIVE")==0) {
			flight_type=0; not_done=0;
		}
	} while(not_done); // Wait for user to choose flight type
	memset(reply,0,sizeof(reply));

	printf("Type [CONNECTED_CONNECTED_CONNECTED!] when you have _c_o_n_n_e_c_t_e_d_ the launchpad battery umbilical: ");
	Treat_reply("CONNECTED_CONNECTED_CONNECTED!");
	printf("Awaiting launch umbilical cord disconnect... "); fflush(stdout);
	while (GPIO_READ(launch_detect_gpio)) { // while ==1 <- pin is HIGH = 3.3 [V]
		// While the rocket is on the launchpad (launchpad battery connected is connected
		// by umbilicals to rocket), wait
	}
	printf("Launch DETECT!\n\n"); fflush(stdout);
	//############################ END OF WAIT FOR LAUNCH ############################

	if (flight_type==1) { // Active flight has been chosen

		//############################ POWERED FLIGHT DATA LOGGING START ############################
		usleep(ENGINE__BURN_TIME); // Wait for engine to burn out
		//############################ POWERED FLIGHT DATA LOGGING END ############################
		printf("\nENGINE BURNOUT! Activating control loop.\n\n");

		//############################ CONTROL LOOP START ############################
		char CONTROL_MESSAGE[200];
		write_to_file_custom(control_log,"time_control_glob \t control_time \t Fpitch \t Fyaw \t Mroll \t R1 \t R2 \t R3 \t R4 \t PWM1 \t PWM2 \t PWM3 \t PWM4\n",error_log);

		gettimeofday(&before_control, NULL);
		gettimeofday(&before_loop, NULL);
		do { // Active control with RCS (Reaction Control System)
			check_time(&now_control_glob,GLOBAL__TIME_STARTPOINT,elapsed_control_glob,&time_control_glob);
			check_time(&now_loop,before_loop,elapsed_loop,&time_loop);

			passive_wait(&now_control,&before_control,&elapsed_control,&time_control,CONTROL__TIME_STEP);

			psi_cont=psi_filt;
			psidot_cont=psi_dot_filt;
			theta_cont=theta_filt;
			thetadot_cont=theta_dot_filt;
			phi_cont=phi_filt;
			wx_cont=wx;

			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% APPLY CONTROL LAW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
			// We calculate Fpitch, Fyaw and Mroll based on a proportional control scheme

			//******************************* Fpitch *******************************
			Fpitch = Fpitch_loop.K*(theta_cont-theta_ref)+Fpitch_loop.Td*thetadot_cont; // PD controller
			//******************************* Fyaw *******************************
			Fyaw = Fyaw_loop.K*(psi_cont-psi_ref)+Fyaw_loop.Td*psidot_cont; // PD controller
			//******************************* Mroll *******************************
			Mroll = Mroll_loop.K*(wx_cont-wx_ref); // P controller

			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMPLEX OPTIMAL THRUST ALLOCATOR %%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
			// Create Simplex parameter matrix
			A[1][1]=0; A[1][2]=-1; A[1][3]=-1; A[1][4]=-1; A[1][5]=-1; // Cost function (negative since we want to minimize), A[1][1] is
																	   // constant term which is zero for cost function
			// Fpitch equality constraint
			if (Fpitch>=0) { A[2][1]=Fpitch; A[2][2]=cos(phi_cont); A[2][3]=-sin(phi_cont); A[2][4]=-cos(phi_cont); A[2][5]=sin(phi_cont); }
			else { A[2][1]=-Fpitch; A[2][2]=-cos(phi_cont); A[2][3]=sin(phi_cont); A[2][4]=cos(phi_cont); A[2][5]=-sin(phi_cont); }
			// Fyaw equality constraint
			if (Fyaw>=0) { A[3][1]=Fyaw; A[3][2]=sin(phi_cont); A[3][3]=cos(phi_cont); A[3][4]=-sin(phi_cont); A[3][5]=-cos(phi_cont); }
			else { A[3][1]=-Fyaw; A[3][2]=-sin(phi_cont); A[3][3]=-cos(phi_cont); A[3][4]=sin(phi_cont); A[3][5]=cos(phi_cont); }
			// Mroll equality constraint
			if (Mroll>=0) { A[4][1]=Mroll; A[4][2]=d; A[4][3]=-d; A[4][4]=d; A[4][5]=-d; }
			else { A[4][1]=-Mroll; A[4][2]=-d; A[4][3]=d; A[4][4]=-d; A[4][5]=d; }

			simplx(A,M,N,M1,M2,M3,&ICASE,IZROV,IPOSV); // Solve linear optimization problem using the Simplex method
			get_simplex_solution(ICASE,IPOSV,A,M,N,&R1,&R2,&R3,&R4); // Push simplex optimal result into the R1, R2, R3, R4 valve thrust variables
			// Saturate valve thrusts to VALVE__MAX_THRUST
			if (R1>=VALVE__MAX_THRUST) R1=VALVE__MAX_THRUST;
			if (R2>=VALVE__MAX_THRUST) R2=VALVE__MAX_THRUST;
			if (R3>=VALVE__MAX_THRUST) R3=VALVE__MAX_THRUST;
			if (R4>=VALVE__MAX_THRUST) R4=VALVE__MAX_THRUST;

			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SEND TO MSP430 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
			// First, convert thrust values to PWM (PWM values 0-127, i.e. 7-bit PWM)
			// TODO : valve open loop control calibration! Don't forget to chance VALVE__MAX_THRUST in valve_curve.m!!!!!!!
			search_PWM(R1,&PWM1);
			search_PWM(R2,&PWM2);
			search_PWM(R3,&PWM3);
			search_PWM(R4,&PWM4);

			// Send PWM values to MSP430
			if (PWM1==0) {
				which_zero=0b00100000;
				MSP430_UART_write_PWM(PWM2,PWM3,PWM4);
			} else if (PWM2==0) {
				which_zero=0b01000000;
				MSP430_UART_write_PWM(PWM1,PWM3,PWM4);
			} else if (PWM3==0) {
				which_zero=0b01100000;
				MSP430_UART_write_PWM(PWM1,PWM2,PWM4);
			} else {
				which_zero=0b10000000;
				MSP430_UART_write_PWM(PWM1,PWM2,PWM3);
			}

			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOG DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

			sprintf(CONTROL_MESSAGE,"%llu\t%llu\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%u\t%u\t%u\t%u\n",time_control_glob,time_control,Fpitch,Fyaw,Mroll,R1,R2,R3,R4,PWM1,PWM2,PWM3,PWM4);
			write_to_file_custom(control_log,CONTROL_MESSAGE,error_log);
			printf("control_time: %llu \t PWM1 : %u \t PWM2 : %u \t PWM3 : %u \t PWM4 : %u \n",time_control,PWM1,PWM2,PWM3,PWM4);
		} while(time_loop<=ACTIVE__CONTROL_TIME);
		which_zero=0b00100000; MSP430_UART_write_PWM(0,0,0); // Send a final transmission to MSP430 microcontroller with 0 PWM values to close the valves
		//############################ CONTROL LOOP END ############################
		printf("\nFINISHED CONTROL LOOP! Data that follows is for rocket descent with parachute (unpowered).\n\n");

		//############################ PARACHUTE DESCENT START ############################
		usleep(DESCENT__TIME);
		//############################ PARACHUTE DESCENT END ############################

	} // if (flight_type==1) (ACTIVE FLIGHT)
	else { // (PASSIVE FLIGHT), i.e. just data recording
		usleep(ENGINE__BURN_TIME+ACTIVE__CONTROL_TIME+DESCENT__TIME); // Put main() thread to sleep while the sensor reading threads execute and log data
	}

	//############################ CLOSING OPERATIONS START ############################
	printf("Flight complete! Exiting GNC program...\n");

	if (flight_type==1) MSP430_UART_write("@e!"); // Command MSP430 to do a software reset if this was an active control flight

	//----------- Quit SPI pressure data reading thread -----------
	SPI_quit=1;
	pthread_join(SPI_pressure_thread,NULL);
	//-------------------------------------------------------------
	//--------------- Quit IMU data reading and filtering threads ----------------
	IMU_quit=1;
	pthread_join(IMU_thread,NULL);
	pthread_join(Filt_thread,NULL);
	//----------------------------------------------------------------------------

	pthread_mutex_destroy(&error_log_write_lock); // All other threads closed now, so destroy error log mutex

	unmap_peripheral(&gpio); // Unmap the GPIO peripheral of the Raspberry Pi

	stopVideo(); // End the Raspberry Pi Spy Camera recording

	fclose(error_log);
	fclose(imu_log);
	fclose(pressure_log);
	fclose(control_log);

	reset_old_attr_port(RAZOR_UART,&old_razor_uart_options);
	close_port(RAZOR_UART);

	printf("All activities shut down. Good-bye!\n");
	//############################ CLOSING OPERATIONS END ##############################
	pthread_exit(NULL);
}
