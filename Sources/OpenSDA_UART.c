/*
 * OpenSDA_UART.c
 * 
 * Support for the UART interface with the OpenSDA microcontroller on the FRDM-KL25Z
 * The  OpenSDA UART is UART0 
 * 
 * Baud rate clock source - See section 5.7.7 (UART Clocking) of the KL25Z Reference Manual 
 * about selection of the baud rate clock source for UART0
 * 
 *  Created on: Nov 11, 2020
 *      Author: sac
 */

#include "derivative.h"
#include "KL25Z_port.h"
#include "OpenSDA_UART.h"

/* enable the bus clock in UART0 */
void enable_opensda_uart_clock(void){
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	return;
}

/* sets up pins for the UART
 */
void init_opensda_uart_pins(void){    
 
	set_port_mux(PORTA_PERIPHERAL, OPENSDA_UART_TX_PIN, OPENSDA_UART_TX_MUX);
	set_port_mux(PORTA_PERIPHERAL, OPENSDA_UART_RX_PIN, OPENSDA_UART_RX_MUX);
	return;
}

/* OpenSDA UART initialization
 * 
 * Lowest baud rate supported is 9600 baud.
 * Data bits are fixed at 8. If parity is enabled, the parity bit is the 8th bit
 * No interrupts enabled here
 * 
 * Baud Rate Clock Source Selection done in the SIM_SOPT2 register. Clock selection
 * must be done when the Tx and Tx are disabled
 * 
 * SIM_SOPT2[PLLFLLSEL]	SIM_SOPT2[UART0SRC[1..0]]	CLOCK SOURCE
 * --------------------	-------------------------	--------------
 * X					00							Clock Disabled
 * 0					01							MCGFLLCLK
 * 1					01							MCGPLLCLK/2
 * X					10							OSCERCLK
 * X					11							MCGIRCLK
 *
 * 
 * 
 * Parameters:
 * 	clock_source	:	selects the baud rate clock source. See enums in the .h file
 *  baud_divisor  	: 	value for the UART1_BDL register
 *  stop_bits		:	0 is 1 stop bit, 1 is 2 stop bits
 *  parity_eable	:	0 is no parity, 1 is parity enabled
 *  parity			:	0 is even parity, 1 is odd parity
 * 	
 */ 

	void init_opensda_uart(uint8_t clock_source, uint8_t baud_divisor, uint8_t stop_bits, uint8_t parity_enable, uint8_t parity_type){    
	
	UART0_C2 = 0;	/* Disable Tx/Rx before selecting baud clock and Rx OSR */ 

	SIM_SOPT2 |= (((uint32_t)clock_source & 0x3) << SIM_SOPT2_UART0SRC_SHIFT)
					| ((((uint32_t)clock_source & 0x4) >> 2) << SIM_SOPT2_PLLFLLSEL_SHIFT);

	UART0_BDH = (stop_bits << UART0_BDH_SBNS_SHIFT) & UART0_BDH_SBNS_MASK;
	UART0_BDL = baud_divisor;
	UART0_C1 = ((parity_enable << UART0_C1_PE_SHIFT) & UART0_C1_PE_MASK)
					| ((parity_type << UART0_C1_PT_SHIFT) & UART0_C1_PT_MASK);
	UART0_C3 = 0; 
	UART0_C4 = (UART0_C4 & ~(UART0_C4_OSR_MASK)) | (OPENSDA_UART_OVERSAMPLING_RATIO << UART0_C4_OSR_SHIFT); 
	UART0_C5 = 0; 
	UART0_C2 = UART0_C2_RE_MASK | UART0_C2_TE_MASK; 
	return;
}

/* returns 1 if the output data register is empty */
uint8_t get_opensda_uart_transmit_data_register_status(void){
	return (UART0_S1 & UART0_S1_TDRE_MASK) >> UART0_S1_TDRE_SHIFT;
}

/* returns 1 if the input data register is full */
uint8_t get_opensda_uart_receive_data_register_status(void){
	return (UART0_S1 & UART0_S1_RDRF_MASK) >> UART0_S1_RDRF_SHIFT;
}

/* Waits for the transmit data register to be empty. Check the transmit data register
 * status before calling this if you are worried about the transmission hanging up
 * or waiting.
 */
void opensda_uart_transmit_char(char char_out){
	while (!((UART0_S1 & UART0_S1_TDRE_MASK) >> UART0_S1_TDRE_SHIFT)) {};	// wait for Tx empty
	UART0_D = char_out;
	return;
}

/* writes a character string in standard format, NULL at the end */
void opensda_uart_transmit_string(char *c)
{
	while (*c){
		opensda_uart_transmit_char(*c);
		c++;
	}
}

/* Waits for the receive data register to be full. Check the receive data register
 * status before calling this if you are worried about receiving hanging up
 * or waiting.
 * If parity is enabled, the parity bit is masked off before returning the data
 */
char opensda_uart_receive_char(void){
	while (!((UART0_S1 & UART0_S1_RDRF_MASK) >> UART0_S1_RDRF_SHIFT)) {};	// wait for Rx data
	return ((UART0_C1 & UART0_C1_PE_MASK) ? UART0_D & 0x7F : UART0_D);
}

