/**
 * @file msp430_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief MSP430 header file
 *
 * This is the header to msp430_funcs.c containing necessary definitions and
 * initializations.
 */

#ifndef MSP430_HEADER_H_
#define MSP430_HEADER_H_

# define MSP430_MAX_BUFFER 1 ///< Buffer size for receving messages from MSP430 (just '!' so 1 byte buffer is used)

char MSP430_RX[MSP430_MAX_BUFFER]; ///< Buffer holding received values via UART from Razor IMU
int MSP430_UART; ///< Holds Razor IMU connection file
char MSP430_reply_string; ///< String holding the MSP430 reply
unsigned char PWM_TX_packet[6]; ///< Packet of 1 byte for "#" and 5 bytes containing the 4 PWM values, to send to MSP430

struct termios new_msp430_uart_options; ///< The new options we set for communicating the the MSP430 UART after opening it.
struct termios old_msp430_uart_options; ///< The old options we save after opening the MSP430 UART connection; we restitute them before closing the connection at the end of the program.

/** @cond INCLUDE_WITH_DOXYGEN */
void MSP430_UART_receive();
void MSP430_UART_write(char MSP430_TX[3]);
void MSP430_UART_write_PWM(unsigned int PWM1, unsigned int PWM2,unsigned int PWM3,unsigned int PWM4);
/** @endcond */

#endif /* MSP430_HEADER_H_ */
