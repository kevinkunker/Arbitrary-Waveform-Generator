/*
 * OpenSDA_UART.h
 * 
 * Supports the UART connection to the OpenSDA microcontroller on the FRDM-KL25Z
 * The OpenSDA UART is UART0
 * 
 *  Created on: Nov 10, 2020
 *      Author: sac
 */

#ifndef OpenSDA_UART_H_
#define OpenSDA_UART_H_

#define OPENSDA_UART_TX_PIN	2
#define OPENSDA_UART_TX_MUX	ALT2
#define OPENSDA_UART_RX_PIN	1
#define OPENSDA_UART_RX_MUX	ALT2

#define OPENSDA_UART_OVERSAMPLING_RATIO	0x0F  // See UART0_C4 OSR[] bits

/* UART0 Baud rate clock source */
enum opensda_uart_baud_clock_source {
	OPENSDA_UART_BAUD_CLOCK_DISABLED = 0,
	OPENSDA_UART_BAUD_CLOCK_MCGIRCLK = 3,
	OPENSDA_UART_BAUD_CLOCK_OSCERCLK = 2,
	OPENSDA_UART_BAUD_CLOCK_MCGFLLCLK = 1,
	OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2 = 5,
};


/* Baud rate divisor, assumes 24MHz bus clock
 * 9600 baud is the lowest rate supported
 * Baud rate divisor = 24MHz/(16 x Baud-Rate)
 */
enum opensda_uart_baud_rate_divisor {
	OPENSDA_UART_BAUD_9600 = 156,
	OPENSDA_UART_BAUD_19200 = 78,
	OPENSDA_UART_BAUD_38400 = 39,
	OPENSDA_UART_BAUD_57600 = 26,
	OPENSDA_UART_BAUD_115200 = 13
};

enum opensda_uart_stop_bits {
	OPENSDA_UART_STOP_BITS_1 = 0,
	OPENSDA_UART_STOP_BITS_2 = 1
};

enum opensda_uart_parity_on_off {
	OPENSDA_UART_PARITY_OFF = 0,
	OPENSDA_UART_PARITY_ON = 1
};

enum opensda_uart_parity_type {
	OPENSDA_UART_PARITY_EVEN = 0,
	OPENSDA_UART_PARITY_ODD = 1
};

void enable_opensda_uart_clock(void);
void init_opensda_uart_pins(void);
void init_opensda_uart(uint8_t clock_source, uint8_t baud_divisor, uint8_t stop_bits,
						uint8_t parity_enable, uint8_t parity_type);    
uint8_t get_opensda_uart_transmit_data_register_status(void);
uint8_t get_opensda_uart_receive_data_register_status(void);
void opensda_uart_transmit_char(char char_out);
void opensda_uart_transmit_string(char *c);
char opensda_uart_receive_char(void);

#endif /* OpenSDA_UART_H_ */
