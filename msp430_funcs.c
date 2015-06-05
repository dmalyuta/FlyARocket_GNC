/**
 * @file msp430_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief MSP430 functions file
 *
 * This file contains functions necessary for communicatin with the MSP430
 * salve microcontroller that is used to do hardware PWM (which the Raspberry Pi
 * is not capable of doing by itself 4 independent times) to drive the 4 RCS
 * valves.
 */

# include <stdio.h>
# include <fcntl.h>
# include <unistd.h>
# include <sys/termios.h> // UART serial communications library
# include <stdint.h>
# include <stdlib.h>
# include <string.h> /* memset */
# include <unistd.h> /* close */
# include <sys/time.h>
# include "msp430_header.h"
# include "master_header.h"

/**
 * @fn void MSP430_UART_receive()
 *
 * This function receives a single byte (over UART) from the MSP430. When this byte is received (it's a BLOCKING read),
 * we know that the MSP430 has successfuly processed the byte we previously sent it and hence is ready to receive
 * another byte.
 *
 * Note : MSP430 sends the byte '!', but note that we do not actually check that the byte equals '!' (0x21)
 * because:
 * 		- The connections has been tested to never fail during numerous pre-tests
 * 		- Even if the byte is not '!' due to signal noise, what can we do? The communication speed is optimised for
 * 		  speed and not robustness with many failsafes - thus we have no way of resending the MSP430 the previous byte
 * 		  in the case that we do not receive '!'
 * 		- As in the above point, we optimised the code for speed, so checking for equality to '!' is an additional time spent.
 */
void MSP430_UART_receive() {
	if ((read(MSP430_UART,MSP430_RX,1))<0) { // Instruction waits for a byte to be received back from MSP430
		perror("Unable to read from MSP430 UART.\n");
		exit(-2); // Exit with failure
	}
}

/**
 * @fn void MSP430_UART_write(char MSP430_TX[3])
 *
 * This function sends the MSP430 a 3 character string which is:
 * 		- "@s!" : arm the MSP430 for PWM generation
 * 		- "@e!" : stop PWM transmission and do a software reset, which puts the MSP430 into a state where
 * 				  it again waits for "@s!"
 *
 * @param MSP430_TX Contains the 3-byte (3-character) string to send to the MSP430
 */
void MSP430_UART_write(char MSP430_TX[3]) {
	int counter=0;
	do {
		if((write(MSP430_UART,&MSP430_TX[counter],1))<0) { // Send MSP430 a byte of the 3-byte command
			perror("Failed to write to the MSP430 UART.\n");
		}
		MSP430_UART_receive(); // Wait for MSP430 to send back "I received the byte that you sent me"
		counter++;
	} while(counter<3);
}

/**
 * @fn void MSP430_UART_write_PWM(unsigned int PWM1, unsigned int PWM2,unsigned int PWM3,unsigned int PWM4)
 *
 * This function sends PWM values to the MSP430.
 * Bit field conversion below:
 *
 * @image latex "MSP430_comm.png" "6 byte packet send by Raspberry Pi to MSP430 to update PWM values" width=15cm
 *
 * In the above image you can see how the 4 10-bit PWM values are distributed across the 6 bytes that are sent to the MSP430.
 * The first byte tells the MSP430 that the following 5 bytes contain PWM values. Once received, the MSP430 decodes these according
 * to the above figure (combining appropriate bits into 10-byte numbers) and assigns them to "unsigned int" type PWM variables that
 * are then output on its 4 pins using timer interrupts (hardware PWM, much more precise than software PWM).
 */
void MSP430_UART_write_PWM(unsigned int PWM1, unsigned int PWM2,unsigned int PWM3,unsigned int PWM4) {
	PWM_TX_packet[0] = '#'; // Tells MSP430 that "the following 5 bytes contain PWM values"
	PWM_TX_packet[1] = ((PWM1&0b1111111100)>>2);
	PWM_TX_packet[2] = ((PWM1&0b0000000011)<<6)|((PWM2&0b1111110000)>>4);
	PWM_TX_packet[3] = ((PWM2&0b00001111)<<4)|((PWM3&0b1111000000)>>6);
	PWM_TX_packet[4] = ((PWM3&0b0000111111)<<2)|((PWM4&0b1100000000)>>8);
	PWM_TX_packet[5] = (PWM4&0b0011111111);

	int counter=0;
	do {
		if((write(MSP430_UART,&PWM_TX_packet[counter],1))<0) { // Send MSP430 a byte
			perror("Failed to write to the MSP430 UART.\n");
		}
		MSP430_UART_receive(); // Wait for MSP430 to send back "I received the byte that you sent me"
		counter++;
	} while(counter<6);
}
