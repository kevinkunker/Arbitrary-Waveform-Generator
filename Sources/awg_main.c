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
#include "KL25Z_gpio.h"

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

/* serial parameters */
#define UART_STREAM_SIZE		20
#define UART_STREAM_TRIG_LEVEL	1
#define RX_MESSAGE_BUFFER_SIZE	20
#define UART0_IRQ_PRIORITY	2

#define DAC_WRITE_BUFFER_LEN 100

#define TIMER_MODULUS   2400000   /* bus clock cycles */

uint32_t dac_write_buffer_1[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t dac_write_buffer_2[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t ARB_buffer_a[100] = {0};                                //ARB Waveform storage a
uint32_t ARB_buffer_b[100] = {0};                                //ARB waveform storage b
uint32_t *write_ptr = dac_write_buffer_1;
uint32_t *read_ptr = dac_write_buffer_2;
uint32_t dac_bit_shift = 0;
StreamBufferHandle_t  UART_Rx_StreamHandle;

/* Function Prototypes */
void Command_Interface_Task(void *pvParameters);

/* Handles */
StreamBufferHandle_t  UART_Rx_StreamHandle;

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
	
	/* Enable PIT interrupt requests in the NVIC (interrupt controller)
	 * Enables interrupt requests from the PIT and sets the IRQ level */
	NVIC_DisableIRQ(PIT_IRQn);
	NVIC_SetPriority(PIT_IRQn, 3);
	NVIC_EnableIRQ(PIT_IRQn);
	
	/* PIT 0 setup with IRQ enabled and start the timer */
	init_PIT(PIT_TIMER_0, TIMER_MODULUS, PIT_INT_ON);
	start_PIT(PIT_TIMER_0);
	
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
	* Enable PTB0 for sync output
	************************************************************/
	init_gpio_pin(GPIOB_PERIPHERAL, 0, 1);
	set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 0);
		
	/************************************************************
	 * should never reach here unless something goes wrong 
    ************************************************************/
		for(;;) {}
		return 0;
	
}

enum CommandStates{COMMAND_IDLE, COMMAND_START, COMMAND_ATTENUATION, COMMAND_FREQUENCY, COMMAND_WAVEFORM_SEL,
	               COMMAND_DAC_WB, COMMAND_RSTATUS, COMMAND_ARB_WR, COMMAND_ARB_RD};


void Command_Interface_Task(void *pvParameters){
	
	extern StreamBufferHandle_t UART_Rx_StreamHandle;   //buffer for reading serial port
	BaseType_t StreamStatus;
	char rx_char, cmd_state;
	
	//enable uart interrupt here?
	
	cmd_state = COMMAND_IDLE;
	for( ;; ){
		
		//receive a character from stream buffer
		StreamStatus = xStreamBufferReceive(UART_Rx_StreamHandle, &rx_char, 1, portMAX_DELAY);
		
		if (StreamStatus != 0){
					
					/* check for a "/" to start command processing */
					switch(cmd_state){
						
					case COMMAND_IDLE :
							/* check for "/" */
							if (rx_char == ('/'))
							{
								cmd_state = COMMAND_START;
							}
							break;
							
					case COMMAND_START :
						 
						    if ((rx_char == ('a')) || (rx_char == ('A'))){          //Set Attenuation
						    	cmd_state = COMMAND_ATTENUATION;
							}
							
						    if ((rx_char == ('f')) || (rx_char == ('F'))){          //Set Frequency
							    cmd_state = COMMAND_FREQUENCY;
							}
							
						    if ((rx_char == ('v')) || (rx_char == ('V'))){          //Select Waveform
						    	cmd_state = COMMAND_WAVEFORM_SEL;
							}
							
						    if ((rx_char == ('b')) || (rx_char == ('B'))){          //Set DAC Write Buffer for output to PIT_IRQHandler
							    cmd_state = COMMAND_DAC_WB;
							}
							
						    if ((rx_char == ('s')) || (rx_char == ('S'))){          //Return Status
								cmd_state = COMMAND_RSTATUS;							
							}
							
						    if ((rx_char == ('w')) || (rx_char == ('W'))){          //ARB data write
						        cmd_state = COMMAND_ARB_WR;
						    }
							
						    if ((rx_char == ('r')) || (rx_char == ('R'))){          //ARB data read
						        cmd_state = COMMAND_ARB_RD;
						    }
						    
						    else{
						    	opensda_uart_transmit_string("/E 1");              //Error message for unrecognized command
						    	cmd_state = COMMAND_IDLE;						   //Idle state if invalid character
						    }
							break;
							
					case COMMAND_ATTENUATION :
						    if (rx_char == (' ')){                                  //Check for space between command and parameter
						    	
						    }
						
							break;
					case COMMAND_FREQUENCY :
						
							break;
					case COMMAND_WAVEFORM_SEL :
						
							break;
					case COMMAND_DAC_WB :
						
							break;
					case COMMAND_RSTATUS :
						
							break;
					case COMMAND_ARB_WR :
						
							break;
					case COMMAND_ARB_RD :
						
							
					
							
					default :
							cmd_state = COMMAND_IDLE;                                           //Default state
							break;
	
							
							
   } //switch case
	
  }	// stream status
	
 } //forever loop
	
} //Command Interface Task

void PIT_IRQHandler(){
	
	static int i = 0;
	
	clear_PIT_int_flag(PIT_TIMER_0);	/* acknowledge the IRQ in the timer*/
	
	switch(i){
		case 0:
			set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 1);
			break;
		case DAC_WRITE_BUFFER_LEN/2:
			set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 0);
			break;
		case DAC_WRITE_BUFFER_LEN:
			i = 0;
			break;
	}
	
	set_dac_output(read_ptr[i++] >> dac_bit_shift);
	return;
}
