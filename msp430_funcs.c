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
 * @fn void MSP430_UART_write_PWM(unsigned char PWMA,unsigned char PWMB,unsigned char PWMC)
 *
 * This function sends PWM values to the MSP430.
 * Bit field conversion below: // TODO : in MSP430 program, add condition "not reading PWM" for # and @ --> avoids that new PWM sent if PWM of 35 is sent!
 *
 * @image latex "MSP430_comm.png" "4 byte packet send by Raspberry Pi to MSP430 to update PWM values" width=15cm
 *
 * 4 bytes are hence sent to the MSP430 where:
 * 	- PWM_TX_packet[0] (byte 1) : send "#" message which tells MSP430 that the following 3 bytes are PWM values
 * 	- PWM_TX_packet[1] (byte 2) : YYY is a code which says:
 * 		* YYY==001 : PWM1 is 0
 * 		* YYY==010 : PWM2 is 0
 * 		* YYY==011 : PWM3 is 0
 * 		* YYY==100 : PWM4 is 0
 * This is because at every point in time only 3 valves are assigned thrusts - this is the consequence of the RCS physics and optimal thrust
 * assignment. It's useless to waste 7 bits sending a 0 value, so we use just 3 to identify which of the PWM values is 0
 *
 * In the above image you can see how the PWMA, PWMB and PWMC signals are distributed amongst the three bytes PWM_TX_packet[1] to PWM_TX_packet[3].
 * In the figure, the MSB is on the left and LSB on the right for each series of A, B and C. We call them PWMA, PWMB and PWMC because these are, in
 * rising order from 1 to 4 the other 3 non-zero PWMs. For example:
 *
 * YYY==001, therefore PWM1 is 0 so PWMA=PWM2, PWMB=PWM2, PWMC=PWM4
 * YYY==011, therefore PWM3 is 0 so PWMA=PWM1, PWMB=PWM2, PWMC=PWM4
 *
 * You see that we simply so from PWM1 to PWM4, skipping the PWM that is 0.
 */
void MSP430_UART_write_PWM(unsigned char PWMA,unsigned char PWMB,unsigned char PWMC) {
	PWM_TX_packet[0] = '#';
	PWM_TX_packet[1] = which_zero | ((PWMA & 0b1111100)>>2);
	PWM_TX_packet[2] = ((PWMA & 0b0000011)<<6) | ((PWMB & 0b1111110)>>1);
	PWM_TX_packet[3] = ((PWMA & 0b0000001)<<7) | PWMC;

	int counter=0;
	do {
		if((write(MSP430_UART,&PWM_TX_packet[counter],1))<0) { // Send MSP430 a byte
			perror("Failed to write to the MSP430 UART.\n");
		}
		MSP430_UART_receive(); // Wait for MSP430 to send back "I received the byte that you sent me"
		counter++;
	} while(counter<4);
}
