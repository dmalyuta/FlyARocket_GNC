/* MSP430 slave file : Fly-A-Rocket GNC
 * Version : v1.0
 * Author : Danylo Malyuta, EPFL 2015
 * Contributors:
 * 		- Gautier Rouaze (RCS mechanical design)
 * 		- Xavier Collaud (Rocket airframe design)
 * 		- Nikolay Mullin (Rocket design supervision)
 * 		- Mikael Gaspar (Launch systems and ground support)
 * 		- Raimondo Pictet (CFD analysis)
 * 		- John Maslov (GPS tracking)
 * 		- Danylo Malyuta (GNC and avionics)
 * Funding : eSpace Space Engineering Center EPFL
 *
 * This file contains the code for the MSP430 slave microcontroller which, once setup, is configured to receive PWM
 * values for the R1, R2, R3 and R4 valves of the FAR rocket and output them as PWM signals to the actual, physical valves.
 */


# include <msp430g2553.h>
# include <string.h>

# define R1 3
# define R2 1
# define R3 4
# define R4 2
# define Buzzer 4
# define R1_ON (P1OUT|=(1<<R1))
# define R1_OFF (P1OUT&=~(1<<R1))
# define R1_TOGGLE (P1OUT^=(1<<R1))
# define R2_ON (P2OUT|=(1<<R2))
# define R2_OFF (P2OUT&=~(1<<R2))
# define R2_TOGGLE (P2OUT^=(1<<R2))
# define R3_ON (P1OUT|=(1<<R3))
# define R3_OFF (P1OUT&=~(1<<R3))
# define R3_TOGGLE (P1OUT^=(1<<R3))
# define R4_ON (P2OUT|=(1<<R4))
# define R4_OFF (P2OUT&=~(1<<R4))
# define R4_TOGGLE (P2OUT^=(1<<R4))
# define BUZZER_ON (P2OUT|=(1<<Buzzer))
# define BUZZER_OFF (P2OUT&=~(1<<Buzzer))
# define BUZZER_TOGGLE (P2OUT^=(1<<Buzzer))

const unsigned int timer_offset=0xFC00; // Time at which timer A starts ==> counter on 8 bits only (255 values)
/* TECHNICAL NOTE:
 * Parker VSO valves that receive the PWM signals here do NOT start at 0% PWM, but rather a little after 50%
 * PWM duty cycle (see page 2 datasheet: http://www.parker.com/literature/Literature%20Files/Precision%20Fluidics%20Division/UpdatedFiles/VSO%20Data%20Sheet_1_19_11.pdf)
 * Therefore even though the PWM is on "8 bits", we set the last bit to 1 automatically - i.e. our PWM "zero point" is at 128 and the Raspberry Pi sends a
 * 7-bit value between 0 and 127 which gets added onto this "zero point"; therefore when the Raspberry Pi sends 127, we get PWM duty cycle = 255 (100%) and the valves
 * are fully open, and when the Raspberry Piu sends 0, we get PWM duty cycle = 128 (50%) and the valves are fully closed as seen in page 2 datasheet figures.
 */
unsigned int R1_PWM=0; // R1 valve PWM value
unsigned int R2_PWM=0; // R2 valve PWM value
unsigned int R3_PWM=0; // R3 valve PWM value
unsigned int R4_PWM=0; // R4 valve PWM value
unsigned int PWMA_decode;
unsigned int PWMB_decode;
unsigned int PWMC_decode;
unsigned char PWM__BUFFER[5]; // Buffer for loading in the PWM values send from Raspberry Pi over UART
unsigned char pwm_ii=0; // Counter for PWM buffer recording, initialized to zero
unsigned char pwm_receiving=0; // Boolean indicating if we are in state (1) of receiving PWM values or in state (0) of not receving PWM signals
unsigned char pwm_convert_now=0; // Boolean indicating (1) I received PWM values, convert them or (0) do not convert anything yet
unsigned long int pwm_timeout_counter=0; // If this value reaches 1172 (~0.15 [s] have passed), reset PWMs to 0
unsigned long int wait_counter=0;
const unsigned char toggle_cycle[2]={11,15}; // Sets number of clock cycles corresponding to one buzzer on/off cycle
							         // Effect : lower toggle_cycle increases PFM frequency for the piezo buzzer
unsigned char toggle_index=0;
const unsigned char volume=8; // At what point to toggle buzzer state in one PFM period
							   // Max value : toggle_cycle
							   // Higher value means high volume (because higher duty cycle)
const unsigned int buzzer_envelope[8] = {1953,
		1709,
		1465,
		1221,
		977,
		732,
		488,
		244
}; // Times for buzzer
unsigned char buzzer_mutliplier = 1; // How many times to repeat a given envelope
unsigned int buzzer_cycle_counter=0;
unsigned int buzzer_envelope_counter=0;

// UART variables
char RX_HANDSHAKE_BUFFER[3]; // Buffer for storing 3-byte message received over UART RX from raspberry pi for handshakes (start/stop)
unsigned char handshake_ii; // counter
unsigned char handshaking; // boolean indicating if a handshake is happening between RPi and MSP430

void UART_quick_reply() {
	while (!(IFG2&UCA0TXIFG)); // USCI_A0 TX buffer ready?
	UCA0TXBUF='!'; // Send character (byte) from reply message
}

void Buzzer_PFM(unsigned int *buzzer_cycle_counter, const unsigned char toggle_cycle, const unsigned char volume) {
	if (*buzzer_cycle_counter==toggle_cycle) {
		*buzzer_cycle_counter=0; // Reset buzzer cycle counter
		BUZZER_TOGGLE; // Toggle the buzzer state (we are generating the PFM...)
	}
}

void Buzzer_warning() {
	int state=0; // 0 := buzzer is OFF
				 // 1 := buzzer is ON and emitting noise
				 // 2 := I want to continue to the next envelope
	int multiply_counter = 0; // Counter for how many times the current envelope has been repeated
	unsigned int i;
	for(i=0;i<7;i++){
		// Loop through buzzer envelope
		while(state!=2) {
			switch (state) {
				case 0:
					BUZZER_OFF; // Buzzer is off...
					if (buzzer_envelope_counter==buzzer_envelope[i]) {
						buzzer_cycle_counter=0; // Reset cycle counter for buzzer, which counts the cycles for PFM
						buzzer_envelope_counter=0; // Reset the envelope counter
						state=1;
					}
					break;
				case 1:
					Buzzer_PFM(&buzzer_cycle_counter,toggle_cycle[toggle_index],volume);
					toggle_index = !toggle_index;
					if(buzzer_envelope_counter==buzzer_envelope[i]) {
						buzzer_cycle_counter=0; buzzer_envelope_counter=0; multiply_counter++;
						if (multiply_counter==buzzer_mutliplier) {
							multiply_counter=0; // Reset multiplier counter
							buzzer_mutliplier++; // Repeat the shorter envelopes ++ more times
							state=2; // I want to continue to the next envelope... ==> will do so because of the while(state!=2) condition!
						} else {
							state=0; // I have not yet repeated the current envelope enough times (as given by buzzer_mutliplier)
									 // Hence I want to repeat it once again, which is equivalent to setting state back to 0
						}
					}
			}
		}
		state=0; // Reset the state to 0, meaning that at the next iteration I will pass into the "buzzer off" state
	}
	// Now play a 2-second long constant tune
	while(state!=2) {
		Buzzer_PFM(&buzzer_cycle_counter,toggle_cycle[toggle_index],volume);
		toggle_index = !toggle_index;
		if(buzzer_envelope_counter==15625) {
			BUZZER_OFF; state=2;
		}
	}
}

void assign_PWM() {
	// This function assigns PWM values received over UART as 5 bytes (storing 40 bits, i.e. 4 PWM values 10 bits each)
	// into 4 distinct 10-bit PWM values that are used for the valve PWM with the timer interrupts

	// Bit field conversion below: see Doxygen/latex/refman.pdf Figure 6.2 to see how the PWM values are decoded from the
	// 5 bytes that are received (note that in that figure PWM_TX_packet[0] was used to identify that PWM values are about
	// to be received and therefore is not stored anywhere, so we have PWM__BUFFER[0]==PWM_TX_packet[1]).

	R1_PWM = (PWM__BUFFER[0]<<2)|((PWM__BUFFER[1]&0b11000000)>>6);
	R2_PWM = ((PWM__BUFFER[1]&0b00111111)<<4)|((PWM__BUFFER[2]&0b11110000)>>4);
	R3_PWM = ((PWM__BUFFER[2]&0b00001111)<<6)|((PWM__BUFFER[3]&0b11111100)>>2);
	R4_PWM = ((PWM__BUFFER[3]&0b00000011)<<8)|PWM__BUFFER[4];

	pwm_convert_now=0; // PWM values have been converted
}

int main(void) {
	// ************* GENERAL CONFIG *************
	WDTCTL=WDTPW+WDTHOLD; // Disable watchdog
	BCSCTL1=CALBC1_16MHZ; DCOCTL=CALDCO_16MHZ; // 16 [MHz] CPU clock speed

	// ************* TIMER CONFIG *************
	TA0CTL=TASSEL_2+ID_1+MC_2; // Set TA0 clock to 8 [MHz] (means that timer overflow at 7812.4 [Hz], so PWM is at 7812.5 [Hz] which is in the 5-12 [kHz] band required for valves), continuous mode, interrupt enable
	TA1CTL=TA0CTL; // Set TA1 clock to 8 [MHz], continuous mode, interrupt enable (same as TA0 clock)
	TA1CCTL2=CCIE; // Enable TA1CCR2 interrupt
				   // Use: R1 valve PWM generation
	TA0CCTL1=CCIE; // Enable TA0CCR1 interrupt
				   // Use: R2 valve PWM generation
	TA0CCTL2=CCIE; // Enable TA0CCR2 interrupt
				   // Use: R3 valve PWM generation
	TA1CCTL1=CCIE; // Enable TA1CCR1 interrupt
				   // Use: R4 valve PWM generation

	// ************* UART CONFIG *************
	P1SEL = BIT1 + BIT2; // P1.1 = RXD, P1.2 = TXD
	P1SEL2 = BIT1 + BIT2;

	UCA0CTL1 |= UCSSEL_2; // SMCLK
	UCA0BR0 = 138;                              // 16MHz 115200
	UCA0BR1 = 0;                                // 16MHz 115200
	UCA0MCTL = UCBRS2 + UCBRS1 + UCBRS0;        // Modulation UCBRSx = 7
	UCA0CTL1 &= ~UCSWRST;                       // **Initialize USCI state machine**C
	IE2 |= UCA0RXIE;                            // Enable USCI_A0 RX interrupt

	__bis_SR_register(GIE); // Enable interrupts

	handshake_ii=0; // Initialize handshake counter
	handshaking=0; // Initialize handshaking boolean

	// Now we wait for command from Raspberry Pi to run the rest of the program
	while(strcmp(RX_HANDSHAKE_BUFFER,"@s!")!=0) { // While we have not received the "start" handshake from Raspberry Pi
		// Waiting...
	}
	// Start handshake has been received if program reaches this line!
	memset(RX_HANDSHAKE_BUFFER,0,3); // Clear the handshake buffer
	// ************* I/O CONFIG *************
	P2OUT&=~(1<<Buzzer); P2DIR|=(1<<Buzzer); // Set Buzzer to output, output zero initially
	// First, play Buzzer warning about start of MSP430 main program
	TA0CTL|=TAIE; TA1CTL|=TAIE; // Enable timer interrupts ==> PWM possible from now on!
	Buzzer_warning(); // We spend close to 7 seconds in the Buzzer_warning() function playing a tune
					  // on the Piezo buzzer to warn user to move away from RCS exit proximity!
	// Second, set our pins and move into the while(1) main loop
	P1OUT&=~(1<<R1)&~(1<<R3); // Set R1,R3 to zero
	P1DIR|=(1<<R1)|(1<<R3); // Enable R1,R3 as outputs
	P2OUT&=~(1<<R2)&~(1<<R4); // Set R2,R4,Buzzer to zero
	P2DIR|=(1<<R2)|(1<<R4); // Enable R2,R4,Buzzer as outputs

	// NB : be aware, LEDs are installed on the rocket avionics to represent valve thrust, but LED brightness perception
	// by the human eye is logarithmic, i.e. NON-linear! Therefore the thrust "levels" are not to be trusted on the LEDs
	// over terminal/log file output of the ACTUAL valve thrusts as the solenoid valves ARE linear with respect to PWM value.

	pwm_timeout_counter=0; // Reset the PWM timeout counter prior to entering the main loop
	while(1) {
		if (pwm_timeout_counter>=1172) { // If no PWM has been received for ~0.15 [s]
			// Then reset PWM values to zero ==> this is a safety agains the program on RPi failing before closing the MSP430 connection, leaving
			// the valves open at some arbitrary position and hence draining the CO2 cartridge!
			R1_PWM=0; R2_PWM=0; R3_PWM=0; R4_PWM=0;
		}
		if (pwm_convert_now) { // If PWM values need to be translated into what MSP430 understands...
			pwm_timeout_counter=0; // As PWM values have been received from the Raspberry Pi, reset the timeout counter for PWM
			assign_PWM(); // ... Then assign the PWM values! These will then be taken into account at the next cycle of the PWM timer!
		}
		if (strcmp(RX_HANDSHAKE_BUFFER,"@e!")==0) { // Means the Raspberry Pi request the MSP430 microcontroller to stop or that no PWM has been received for 0.2 seconds
			memset(RX_HANDSHAKE_BUFFER,0,3); // Clear the handshake buffer
			wait_counter=0;
			while (wait_counter<3000) {
				// Wait some time
			}
			break; // Break out of while(1) loop
		}
	}
	WDTCTL = 0; // Software reset (start program from beginning)
}

/**************************************************************
 *************************** TIMER A0 *************************
 **************************************************************/
// TA0CCR1, TA0CCR2, TA0 overflow interrupts
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1(void) {
	switch(TA0IV) {
		case  2: // TA0CCR1 routine
			R2_OFF;
			break;
		case  4: // TA0CCR2 routine
			R3_OFF;
			break;
		case 10: // TA0 overflow routine
			wait_counter++; pwm_timeout_counter++; buzzer_envelope_counter++; buzzer_cycle_counter++;
			TA0R=timer_offset; // Offset TA0R register value to count on 10 remaining bits only (giving 1024 value resolution)
			TA0CCR1=timer_offset+R2_PWM; // Set R2 valve PWM interrupt
			if (R2_PWM) R2_ON; // Only turn on R2 if its PWM is non-zero
			TA0CCR2=timer_offset+R3_PWM; // Set R3 valve PWM interrupt
			if (R3_PWM) R3_ON; // Only turn on R3 if its PWM is non-zero
			break;
	}
}

/**************************************************************
 *************************** TIMER A1 *************************
 **************************************************************/
// TA1CCR1, TA1CCR2, TA1 overflow interrupts
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1(void) {
	switch(TA1IV) {
		case  2: // TA1CCR1 routine
			R4_OFF;
			break;
		case  4: // TA1CCR2 routine
			R1_OFF;
			break;
		case 10: // TA1 overflow routine
			TA1R=timer_offset; // Offset TA1R register value to count on 10 remaining bits only (giving 1024 value resolution)
			TA1CCR1=timer_offset+R4_PWM; // Set R3 valve PWM interrupt
			if (R4_PWM) R4_ON; // Only turn on R3 if its PWM is non-zero
			TA1CCR2=timer_offset+R1_PWM; // Set R1 valve PWM interrupt
			if (R1_PWM) R1_ON; // Only turn on R1 if its PWM is non-zero
			break;
	}
}

/**************************************************************
 **************** USCI MODULE (for UART) **********************
 **************************************************************/
// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	if (UCA0RXBUF==0x40 && !pwm_receiving && !handshaking) { // ASCII '@' character has been received
		// Start of a handshake with Raspberry pi
		handshaking=1; // Take note that a handshake is happening
		RX_HANDSHAKE_BUFFER[handshake_ii]=UCA0RXBUF;
		P1OUT^=(1<<0); // Switch LED state
		handshake_ii++;
	} else if (handshaking) {
		// Record more of the handshake message
		RX_HANDSHAKE_BUFFER[handshake_ii]=UCA0RXBUF;
		handshake_ii++;
		if (handshake_ii==3) { // I decided that the handshakes are 3 bytes long : "@s!" for "MSP430 START" and "@e!" for "MSP430 END"
							   // Which means when handshake_ii==3 (FOURTH byte), we finish the handshake
			handshake_ii=0; // Reset handshake counter
			handshaking=0; // Handshaking done
		}
	}

	if (UCA0RXBUF==0x23 && !pwm_receiving && !handshaking) { // ASCII '#' character has been received
		// This means the Raspberry pi is about to send a a new set of 4 valve PWMs
		pwm_receiving=1; // We put ourselves into the move "receving PWM values from Raspberry Pi"
		pwm_ii=0; // Reset PWM buffer element counter
	} else if (pwm_receiving) {
		PWM__BUFFER[pwm_ii]=UCA0RXBUF; // Record received value
		pwm_ii++; // Increment counter
		if (pwm_ii>=5) { // If maximum buffer size reached for PWM__BUFFER
			pwm_ii=0; // Reset counter
			pwm_receiving=0; // Finished receiving PWM values
			pwm_convert_now=1; // Request to convert received 5 bytes into 4 10-bit PWM values
		}
	}

	UART_quick_reply(); // Let Raspberry Pi know that the byte has been read by sending '!' character back tot he Raspberry Pi
}
