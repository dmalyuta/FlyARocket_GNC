/**
 * @file master_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Master functions file.
 *
 * This file contains some of the primary, "every-day" functions used by the GNC
 * algorithm to achieve its tasks. The file is called "master_funcs" because the
 * functions here are general in that they tend to appear everywhere in the code
 * to perform routine tasks like file I/O, waiting, etc.
 */

# include <sys/time.h>
# include <stdio.h>
# include <string.h>
# include <stdlib.h>
# include <pthread.h>
# include <unistd.h>
# include "master_header.h"
# include "spycam_header.h"

/**
 * @name Thrust curve group
 * These variables have been collected in experimental open-loop tests to determine what thrust value the valves output for a given PWM (in #PWM_valve_charac).
 * @{
 */
unsigned int PWM_valve_charac[VALVE_CHARAC_RESOLUTION] = {310,420,520,620,720,820,920,1020}; ///< PWM value of characteristic thrust curve
double R_valve_charac[VALVE_CHARAC_RESOLUTION] = {0.0,0.17,0.25,0.29,0.32,0.34,0.35,0.36}; ///< Thrust value [N],relative, of characteristic thrust value for RCS valves
/** @} */

/**
 * @fn void write_to_file_custom(FILE *file_ptr, char *string,FILE *error_log)
 *
 * This function allows to write a custom string to a file
 *
 * @param file_ptr Pointer to file.
 * @param string The file path.
 * @param error_log Pointer to error log file.
 */
void write_to_file_custom(FILE *file_ptr, char *string,FILE *error_log) {
	if ((fprintf(file_ptr,"%s", string))<0) {
		sprintf(ERROR_MESSAGE,"Could not write to custom file!");
		perror(ERROR_MESSAGE); fflush(stdout);
		if (file_ptr!=error_log) { // Make sure the error is not writing to the error log, otherwise we'd have an infinite loop in a recursive
								   // function!
			pthread_mutex_lock(&error_log_write_lock); // Protect writing into error log by more than one thread
			write_to_file_custom(error_log,ERROR_MESSAGE,error_log);
			pthread_mutex_unlock(&error_log_write_lock);
		}
		stopVideo();
		exit(-2); // Exit with a critical failure
	}
}

/**
 * @fn void check_time(struct timeval *now, struct timeval before, struct timeval elapsed, unsigned long long int *time)
 *
 * This function the value pointed to by time with the time that has passed between *now and the last time
 * gettimeofday(&before,NULL) was called.
 *
 * @param now The current time (struct timeval).
 * @param before The previous time (struct timeval).
 * @param elapsed The time elapsed between now and before (struct timeval).
 * @param time The time in [us] that elapsed between now and before.
 */
void check_time(struct timeval *now, struct timeval before, struct timeval elapsed, unsigned long long int *time) {
	gettimeofday(now, NULL);
	if (before.tv_usec > (*now).tv_usec) { // Account for overflow on the .tv_usec timer
		(*now).tv_usec += 1000000;
		(*now).tv_sec--;
	}
	elapsed.tv_usec = (*now).tv_usec - before.tv_usec; // microsecond difference
	elapsed.tv_sec = (*now).tv_sec - before.tv_sec; // second difference
	*time = elapsed.tv_sec*1000000 + elapsed.tv_usec; // Microsecond precision elapsed time since (first)
}

/**
 * @fn void open_file(FILE **log, char *path, char *setting,FILE *error_log)
 *
 * This function opens a file at *path in the mode *setting. If unsuccessful,
 * it writes the error into the error_log (which may itself be unsuccessful and throws
 * an error.
 *
 * @param log This is the pointer to the file we want to open.
 * @param path This is the path to the file.
 * @param setting This is the mode in which we want to open the file (e.g. 'w', write only).
 * @param error_log Pointer to the error log, used in case of error opening log (argument 1).
 */
void open_file(FILE **log, char *path, char *setting,FILE *error_log) {
	if ((*log=fopen(path,setting))==0) {
		sprintf(ERROR_MESSAGE,"CRITICAL ERROR: could not fopen() %s\n",path);
		perror(ERROR_MESSAGE); fflush(stdout);
		pthread_mutex_lock(&error_log_write_lock);
		write_to_file_custom(error_log,ERROR_MESSAGE,error_log);
		pthread_mutex_unlock(&error_log_write_lock);
		stopVideo();
		exit(-2); // Exit with a critical failure
	}
}

/**
 * @fn void open_error_file(FILE **error_log,char *path, char *setting)
 *
 * This file opens the error file. The function is separate from open_file() because
 * in case of error we can't write to the error file.
 *
 * @param error_log Pointer to the error log.
 * @param path This is the path to the error log.
 * @param setting This is the mode in which we want to open the error log (e.g 'w', write only).
 */
void open_error_file(FILE **error_log,char *path, char *setting) {
	if ((*error_log=fopen(path,setting))==0) {
		sprintf(ERROR_MESSAGE,"CRITICAL ERROR: could not fopen() %s",path);
		perror(ERROR_MESSAGE); fflush(stdout);
		stopVideo();
		exit(-2); // Exit with a critical failure
	}
}

/**
 * @fn void passive_wait(struct timeval *now,struct timeval *before,struct timeval *elapsed,unsigned long long int *time,unsigned long long int TIME__STEP)
 *
 * This function does a passive (instead of a busy) wait.
 *
 * @param now The time now (struct timeval).
 * @param before The time before (struct timeval).
 * @param elapsed The time elapsed (struct timeval).
 * @param time The time in [us] between now and before.
 * @param TIME__STEP The time we want to iterate at within the loop from which passive_wait() was called.
 */
void passive_wait(struct timeval *now,struct timeval *before,struct timeval *elapsed,unsigned long long int *time,unsigned long long int TIME__STEP) {
	check_time(now,*before,elapsed_control,time);

	if (TIME__STEP>*time) {
		usleep(TIME__STEP-*time);
	}

	check_time(now,*before,elapsed_control,time);
	*before=*now;
}

/**
 * @fn void search_PWM(double R1_thrust,double R2_thrust,double R3_thrust,double R4_thrust,unsigned int *pwm1,unsigned int *pwm2,unsigned int *pwm3,unsigned int *pwm4)
 *
 * This function, given a wanted thrust, assigns the required PWM to produce that
 * thrust given the PWM_valve_charac[] and R_valve_charac[] arrays which assigns the correct
 * PWM for a given thrust level. Typical thrust curves can be seen on the first
 * figure at page 2 of datasheet found <a href="http://www.parker.com/literature/Literature%20Files/Precision%20Fluidics%20Division/UpdatedFiles/VSO%20Data%20Sheet_1_19_11.pdf">here</a>.
 * However, we manually measured the thrust level for a given PWM using a balance. The data was collected into a spreadsheet
 * and the following graph was produced:
 *
 * @image latex "valve_curve.jpg" "Valve thrust curves" width=15cm
 *
 * The valves in our application are controlled in open-loop due to space and time constraints on implementing
 * sensors to close the loop on valve control. This is suboptimal, of course, due to valves heating up, cooling down,
 * hysteresis, etc. that would slightly make the thrust curve change during flight.
 *
 * @param R1_thrust The thrust we want the valve R1 to output.
 * @param R2_thrust The thrust we want the valve R2 to output.
 * @param R3_thrust The thrust we want the valve R3 to output.
 * @param R4_thrust The thrust we want the valve R4 to output.
 * @param pwm1 The pointer to the PWM1 value (for valve R1).
 * @param pwm2 The pointer to the PWM2 value (for valve R2).
 * @param pwm3 The pointer to the PWM3 value (for valve R3).
 * @param pwm4 The pointer to the PWM4 value (for valve R4).
 */
void search_PWM(double R1_thrust,double R2_thrust,double R3_thrust,double R4_thrust,unsigned int *pwm1,unsigned int *pwm2,unsigned int *pwm3,unsigned int *pwm4) {
	if (R1_thrust!=0) {
		linear_search(R1_thrust,pwm1);
	} else {
		*pwm1 = 0;
	}

	if (R2_thrust!=0) {
		linear_search(R2_thrust,pwm2);
	} else {
		*pwm2=0;
	}

	if (R3_thrust!=0) {
		linear_search(R3_thrust,pwm3);
	} else {
		*pwm3=0;
	}

	if (R4_thrust!=0) {
		linear_search(R4_thrust,pwm4);
	} else {
		*pwm4=0;
	}
}

/**
 * @fn void linear_search(double thrust, unsigned int *pwm)
 * Interpolates discrete thrust curve to give a PWM that produces a given valve thrust.
 *
 * @param thrust Desired valve thrust, in [N].
 * @param pwm Pointer to the memory block holding the pwm value which we'd like to assign.
 */
void linear_search(double thrust, unsigned int *pwm) {
	int zz;
	for (zz=1;zz<VALVE_CHARAC_RESOLUTION;zz++) {
		if (R_valve_charac[zz-1]<=thrust && R_valve_charac[zz]>=thrust) {
			// Interpolate the necessary PWM4 for the given thrust R4
			*pwm = PWM_valve_charac[zz-1]+(unsigned int)(((double)(PWM_valve_charac[zz]-PWM_valve_charac[zz-1]))/(R_valve_charac[zz]-R_valve_charac[zz-1])*(thrust-R_valve_charac[zz-1]));
			break;
		}
	}
}
