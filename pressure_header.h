/**
 * @file pressure_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Honeywell pressure/temperature sensors header file
 *
 * This is the header to pressure_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef PRESSURE_HEADER_H_
#define PRESSURE_HEADER_H_

extern const char RADIAL_SENSOR[];
extern const char AXIAL_SENSOR[];

# define BYTE_NUMBER 4 ///< How many bytes we want to receive from the pressure sensor per reading

/**
 * @struct SPI_data
 * This structure contains all info necessary to communicate with and to interpret incoming data from the Honeywell HSC sensors. Please refer
 * to the <a href="http://sensing.honeywell.com/honeywell-sensing-trustability-hsc-series-high-accuracy-board-mount-pressure-sensors-50099148-a-en.pdf">datasheet</a>
 * and <a href="http://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf">Honeywell SPI companion</a> for info
 * on parameters like P__OUT_MAX, P_OUT__MIN, etc. Note that the specific pressure sensors that we use are: HSC D LN N 100MD S A 5.
 */
struct SPI_data {
	unsigned char mode; ///< SPI mode
	unsigned char bits; ///< Number of bits per SPI transmission
	unsigned long int max_speed; ///< Frequency of SPI transmission (in [Hz])
	unsigned char buffer_length; ///< Length of SPI transmit/receive buffer

	unsigned int P_OUT__MAX; ///< Maximum decimal value received when recording max pressure, refer to <a href="http://sensing.honeywell.com/honeywell-sensing-trustability-hsc-series-high-accuracy-board-mount-pressure-sensors-50099148-a-en.pdf">datasheet</a> page 13 for second to last model number ("Transfer Function") given HSC D LN N 100MD S A 5
	unsigned int P_OUT__MIN; ///< Minimum decimal value received when recording min pressure, refer to <a href="http://sensing.honeywell.com/honeywell-sensing-trustability-hsc-series-high-accuracy-board-mount-pressure-sensors-50099148-a-en.pdf">datasheet</a> page 13 for second to last model number ("Transfer Function") given HSC D LN N 100MD S A 5
	float P__MAX; ///< [mbar] maximum sensor pressure reading
	float P__MIN; ///< [mbar] minimum sensor pressure reading

	unsigned int radial_sensor_fd; ///< Radial sensor connection handle
	unsigned int axial_sensor_fd; ///< Axial sensor connection handle
};

struct SPI_data SPI_config; ///< Holds the SPI configuration

unsigned char radial_status; ///< Holds status of radial sensor
float radial_pressure; ///< Holds differential pressure reading of radially mounted pressure sensor
float radial_temperature; ///< Holds compensated temperature reading of radially mounted pressure sensor

char axial_status; ///< Holds status of axial sensor
float axial_pressure; ///< Holds differential pressure reading of axially mounted pressure sensor
float axial_temperature; ///< Holds compensated temperature reading of axially mounted pressure sensor

unsigned char data[BYTE_NUMBER]; ///< We will receive 4 bytes from the pressure sensor
struct spi_ioc_transfer transfer[BYTE_NUMBER]; ///< SPI transfer structure

/** @cond INCLUDE_WITH_DOXYGEN */
void pressure_sensor_SPI_connect(const char *directory,unsigned int *fd,unsigned char mode, unsigned char bits, unsigned long int max_speed);
void *get_readings_SPI_parallel(void *args);
/** @endcond */

#endif /* PRESSURE_HEADER_H_ */
