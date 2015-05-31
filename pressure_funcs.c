/**
 * @file pressure_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Honeywell pressure/temperature sensors functions file
 *
 * This file contains functions that log data from the Honeywell HSC D LN N 100MD S A 5
 * pressure/temperature sensors (HSC - High Accuracy, Compensated/Amplified - TruStability Series)
 * over SPI communication.
 */

# include <stdio.h>
# include <fcntl.h>
# include <unistd.h>
# include <sys/ioctl.h>
# include <stdint.h>
# include <stdlib.h>
# include <string.h>
# include <linux/spi/spidev.h>
# include <pthread.h>
# include "pressure_header.h"
# include "master_header.h"

const char RADIAL_SENSOR[] = "/dev/spidev0.0"; ///< File path for the radial pressure sensor SPI connection
const char AXIAL_SENSOR[] = "/dev/spidev0.1"; ///< File path for the axial pressure sensor SPI connection

/**
 * @fn void pressure_sensor_SPI_connect(const char *directory,unsigned int *fd,unsigned char mode, unsigned char bits, unsigned long int max_speed)
 *
 * This function opens the SPI connection to the pressure sensor connected to *directory.
 *
 * @param directory The file path.
 * @param fd The connection handle once opened.
 * @param mode SPI mode (0, 1, 2 or 3). Honeywell HSC pressure sensors use mode 0, so that's what we use in this program!
 * @param bits Number of bits per SPI transmission.
 * @param max_speed The SPI connection speed.
 */
void pressure_sensor_SPI_connect(const char *directory,unsigned int *fd,unsigned char mode, unsigned char bits, unsigned long int max_speed) {
	unsigned char error=0; // Boolean which =1 if an error was produced

	memset(ERROR_MESSAGE,0,200); // Clear the error message

	if((*fd = open(directory, O_RDWR)) < 0) {
		perror("SPI: Can't open device.\n");
		error=1;
	}

	if(ioctl(*fd, SPI_IOC_WR_MODE, &mode) == -1) {
		perror("SPI: SPI_IOC_WR_MODE can't set SPI mode.\n");
		error=1;
	}
	if(ioctl(*fd, SPI_IOC_RD_MODE, &mode) == -1) {
		perror("SPI: SPI_IOC_RD_MODE can't get SPI mode.\n");
		error=1;
	}
	if(ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
		perror("SPI: SPI_IOC_WR_BITS_PER_WORD can't set bits per word.\n");
		error=1;
	}
	if(ioctl(*fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
		perror("SPI: SPI_IOC_RD_BITS_PER_WORD can't get bits per word.\n");
		error=1;
	}
	if(ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed) == -1) {
		perror("SPI: SPI_IOC_WR_MAX_SPEED_HZ can't set max speed (in Hz).\n");
		error=1;
	}
	if(ioctl(*fd, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed) == -1) {
		perror("SPI: SPI_IOC_RD_MAX_SPEED_HZ can't get max speed (in Hz).\n");
		error=1;
	}

	if (error) { // If an errors were produced
		exit(-2); // Exit program
	}
}

/**
 * @fn void *get_readings_SPI_parallel(void *args)
 *
 * This is a (p)thread which does the sole job of reading data from the Honeywell HSC sensors (pressure and
 * temperature).
 *
 * @param args A pointer to the input arguments. We pass the SPI connection struct pointer as a void pointer and then typecast it back to a struct pointer (see <a href="https://computing.llnl.gov/tutorials/pthreads/samples/hello_arg2.c">example</a>).
 */
void *get_readings_SPI_parallel(void *args) {
	// Process inputs
	struct SPI_data *my_data;
	my_data = (struct SPI_data *) args;

	unsigned int radial_sensor_fd = (*my_data).radial_sensor_fd; // Equivalent : my_data->radial_sensor_fd
	unsigned int axial_sensor_fd = (*my_data).axial_sensor_fd;
	unsigned char buffer_length = (*my_data).buffer_length;

	unsigned int P_OUT__MAX = (*my_data).P_OUT__MAX;
	unsigned int P_OUT__MIN = (*my_data).P_OUT__MIN;
	float P__MAX = (*my_data).P__MAX;
	float P__MIN = (*my_data).P__MIN;

	unsigned int radial_pressure_output;
	unsigned int radial_temperature_output;
	unsigned int axial_pressure_output;
	unsigned int axial_temperature_output;

	char PRESSURE_WRITE[200];
	write_to_file_custom(pressure_log,"time_pressure_glob \t radial_status \t radial_pressure \t radial_temperature \t axial_status \t axial_pressure \t axial_temperature\n",error_log);

	gettimeofday(&before_pressure, NULL); // Get initial read time
	do {
		check_time(&now_pressure_glob,GLOBAL__TIME_STARTPOINT,elapsed_pressure_glob,&time_pressure_glob);

		passive_wait(&now_pressure,&before_pressure,&elapsed_pressure,&time_pressure,SPI__READ_TIMESTEP);

		//******************************** Read RADIAL pressure sensor ********************************
		if ((ioctl(radial_sensor_fd,SPI_IOC_MESSAGE(buffer_length),transfer))<0) { // Error in SPI communication
			perror("SPI: SPI_IOC_MESSAGE Failed!");
			write_to_file_custom(error_log,"SPI: SPI_IOC_MESSAGE Failed!",error_log);
			exit(-2);
		}
		//-------------------- Now convert received bits into pressure and temperature readings
		// See (http://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf) Figure 3. (on page 2) for decoding of bytes into status,pressure,temperature below
		radial_status = (data[0] & 0b11000000)>>6; // 2 MSB bits of first byte (status written on 2 bits)
		radial_pressure_output = ((data[0] & 0b00111111)<<8) | data[1]; // 6 LSB bits of first byte and all bits of second byte (differential pressure 14 bits resolution)
		radial_temperature_output = (data[2]<<3) | ((data[3] & 0b11100000)>>5); // third byte and 3 MSB bits of fourth byte (compensated temperature 11 bits resolution)

		// Now convert p_output and t_output into a pressure [mbar] and a temperature [°C]
		radial_pressure = ((float)(radial_pressure_output-P_OUT__MIN))*((float)(P__MAX-P__MIN))/((float)(P_OUT__MAX-P_OUT__MIN))+P__MIN; // Pressure reading in [mbar]
		radial_temperature = (((float)(radial_temperature_output))/2047.0)*200.0-50.0; // Temperature reading in [°C]

		//******************************** Read AXIAL pressure sensor ********************************
		if ((ioctl(axial_sensor_fd,SPI_IOC_MESSAGE(buffer_length),transfer))<0) { // Error in SPI communication
			perror("SPI: SPI_IOC_MESSAGE Failed!");
			write_to_file_custom(error_log,"SPI: SPI_IOC_MESSAGE Failed!",error_log);
			exit(-2);
		}
		//-------------------- Now convert received bits into pressure and temperature readings
		axial_status = (data[0] & 0b11000000)>>6; // 2 MSB bits of first byte (status written on 2 bits)
		axial_pressure_output = ((data[0] & 0b00111111)<<8) | data[1]; // 6 LSB bits of first byte and all bits of second byte (differential pressure 14 bits resolution)
		axial_temperature_output = (data[2]<<3) | ((data[3] & 0b11100000)>>5); // third byte and 3 MSB bits of fourth byte (compensated temperature 11 bits resolution)

		// Now convert p_output and t_output into a pressure [mbar] and a temperature [°C]
		axial_pressure = ((float)(axial_pressure_output-P_OUT__MIN))*((float)(P__MAX-P__MIN))/((float)(P_OUT__MAX-P_OUT__MIN))+P__MIN; // Pressure reading in [mbar]
		axial_temperature = (((float)(axial_temperature_output))/2047.0)*200.0-50.0; // Temperature reading in [°C]


		sprintf(PRESSURE_WRITE,"%llu\t%d\t%.5f\t%.5f\t%d\t%.5f\t%.5f\n",time_pressure_glob,radial_status,radial_pressure,radial_temperature,axial_status,axial_pressure,axial_temperature);
		write_to_file_custom(pressure_log,PRESSURE_WRITE,error_log);
	} while(!SPI_quit); // Continue reading sensor until quit

	printf("\nQuitting SPI pressure sensor reading thread!\n");
	pthread_exit(NULL); // Quit the pthread
}
