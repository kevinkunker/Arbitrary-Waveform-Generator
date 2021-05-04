/*
 * awg_main.c
 *
 *  Created on: Apr 27, 2021
 *      Author: Owner
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "derivative.h"
#include "OpenSDA_UART.h"
#include "mcg.h"
#include "KL25Z_port.h"
#include "analog.h"
#include "KL25Z_NVIC.h"
#include "KL25Z_pit.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "stream_buffer.h"

const uint32_t sine_data[] = {
#include "sine_data_100_points.txt"
};

const uint32_t triangle_data[] = {
#include "triangle_data_100_points.txt"
};

#define UART0_IRQ_PRIORITY	2

uint32_t dac_write_buffer_1[100] = {0};
uint32_t dac_write_buffer_2[100] = {0};
uint32_t ARB_buffer_a[100] = {0};                                //ARB Waveform storage a
uint32_t ARB_buffer_b[100] = {0};                                //ARB waveform storage b
uint32_t *write_ptr = dac_write_buffer_1;
uint32_t *read_ptr = dac_write_buffer_2;
uint32_t dac_bit_shift = 0;

int main(void){
	/************************************************************
	 * Set the main clock to 48 MHz 
	 ************************************************************/
	pll_init(8000000,0,1,4,24,1);
	
	/************************************************************
	 * Enable peripheral clocks 
	 ************************************************************/
	enable_port_clock(PORTA_PERIPHERAL);
	enable_port_clock(PORTB_PERIPHERAL);
	enable_port_clock(PORTC_PERIPHERAL);
	enable_port_clock(PORTD_PERIPHERAL);
	enable_port_clock(PORTE_PERIPHERAL);
	enable_opensda_uart_clock();
	enable_pit_clock();
	enable_dac_clock();

	/************************************************************
	 *  Startup the OpenSDA UART (UART0)
	 ************************************************************/
	init_opensda_uart_pins();
	init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_57600,
    				OPENSDA_UART_STOP_BITS_1, OPENSDA_UART_PARITY_OFF, OPENSDA_UART_PARITY_EVEN);
	
	/* send out a welcome message */
	opensda_uart_transmit_string("ELT3050 FreeRTOS ADC Experiments \r\n");

	/* UART0 IRQ settings */
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, UART0_IRQ_PRIORITY);
	NVIC_EnableIRQ(UART0_IRQn);

	/************************************************************
	 * startup the DAC, ADC and hardware timer
	 ************************************************************/
	init_dac();
	init_dac_pin();
	
	/************************************************************
	* command interface task
     ************************************************************/
	xTaskCreate(Command_Interface_Task,				    /* Pointer to the function that implements the task. */
				"Command Interface Task",				/* Friendly name for debugging. */
				100, 					                /* Stack depth (may need to be increased) */
				NULL, 					                /* Task parameter, not used */
				1,						                /* Task priority */
				NULL);					                /* task handle, not used */
	
	/************************************************************
     * create semaphores, queues, streams, etc.
     ************************************************************/
	   //add stream buffer for serial port
	UART_Rx_StreamHandle = xStreamBufferCreate(UART_STREAM_SIZE, UART_STREAM_TRIG_LEVEL);
	
	/************************************************************
	 * start the scheduler. Should never return
	 ************************************************************/
	vTaskStartScheduler();	
		
	/************************************************************
	 * should never reach here unless something goes wrong 
    ************************************************************/
		for(;;) {}
		return 0;
	
}


void Command_Interface_Task(void *pvParameters){
	
	extern StreamBufferHandle_t UART_Rx_StreamHandle;   //buffer for reading serial port
	
	
}

void PIT_IRQHandler(){
	static int i = 0;
	set_dac_output(read_ptr[i] >> dac_bit_shift);
	
}
