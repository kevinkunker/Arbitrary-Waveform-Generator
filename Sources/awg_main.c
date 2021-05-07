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
#define UART0_IRQ_PRIORITY	3

#define DAC_WRITE_BUFFER_LEN 100
#define STRING_LENGTH 10 

#define TIMER_MODULUS   2400000   /* bus clock cycles */

uint32_t dac_write_buffer_1[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t dac_write_buffer_2[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t ARB_buffer_a[DAC_WRITE_BUFFER_LEN] = {0};                                //ARB Waveform storage a
uint32_t ARB_buffer_b[DAC_WRITE_BUFFER_LEN] = {0};                                //ARB waveform storage b
uint32_t *write_ptr = dac_write_buffer_1;
uint32_t *read_ptr = dac_write_buffer_2;
uint32_t dac_bit_shift = 0;

uint32_t parameter_value;                                                         //parameter value to pass into command functions
uint32_t data_value;                                                              //data value for ARB buffer parameter

char serial_in[STRING_LENGTH];													  //Serial input string
char *ser_in = serial_in;                                                         //Pointer to serial_in string

enum CommandStates{COMMAND_IDLE, COMMAND_START};                                  //state machine enumerations

/* Function Prototypes */
void Command_Interface_Task(void *pvParameters);
void PIT_IRQHandler(void);
void UART0_IRQHandler(void);

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
	NVIC_SetPriority(PIT_IRQn, 2);
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

void Command_Interface_Task(void *pvParameters){
	
	extern StreamBufferHandle_t UART_Rx_StreamHandle;   //buffer for reading UART0_IRQHandler
	extern char serial_in[STRING_LENGTH];
	extern uint32_t parameter_value;
	extern uint32_t data_value;
	
	BaseType_t StreamStatus;
	
	char rx_char, cmd, cmd_state;
	
	char temp[STRING_LENGTH] = {'\0'};         //buffer for reversing string
	char *tmp = temp;                          //pointer for temp buffer
	
	char data[STRING_LENGTH] = {'\0'};         //buffer for data parameter
	char *dat = data;                          //pointer for data buffer
	
	
	static int i = 0;
	static int j,k,l;
	static int scount = 0;                              //space counter variable
	
	UART0_C2 |= UART0_C2_RIE_MASK;                      //Enable UART0 interrupt
	
	cmd_state = COMMAND_IDLE;                           //Initialize state machine
	
	for( ;; ){
		
		//receive a character from stream buffer
		StreamStatus = xStreamBufferReceive(UART_Rx_StreamHandle, &rx_char, 1, portMAX_DELAY);
		
		if (StreamStatus != 0){
					
					
					switch(cmd_state){
						
					case COMMAND_IDLE :
						/* check for a "/" to start command processing */
							if (rx_char == ('/'))
							{   
								cmd_state = COMMAND_START;
							}
							break;
							
					case COMMAND_START :                              
						//build a string from serial input
							
							while (rx_char != '\n'){                   //read in string
								
								serial_in[i] = rx_char;
								i++;
							}
							serial_in[i+1] = '\0';					   //terminate string
							
				
						//Parse out command and parameter(s)
							
							for (k=0; serial_in[k] !='\0'; k++){
								
								if (scount == 0 && serial_in[k] != ' ' ){
									cmd = serial_in[k];              //get the command value
								}
								if (scount == 1 && serial_in[k] >= '0' && serial_in[k] <= '9'){
									*tmp = serial_in[k];             //get the parameter value
									 tmp++;                          //put in temporary buffer
								}
								if (scount == 2 && serial_in[k] >= '0' && serial_in[k] <= '9'){
									*dat = serial_in[k];             //get the data value (if needed)
									 dat++;                          //put in temporary buffer
								}
								if (serial_in[k] == ' '){            //if a space character is found
									scount = scount + 1;
								}
							}
							    
							 
						//convert ascii strings to integers for parameter / data use
							
							for (j=0; j <= STRING_LENGTH; j++){
								if(temp[j] >= '0' && temp[j] <= '9'){
									
									parameter_value = parameter_value * 10;
									parameter_value = parameter_value + temp[j] - '0';
								}
							
								if(data[j] >= '0' && data[j] <= '9'){
																
									data_value = data_value * 10;
									data_value = data_value + data[j] - '0';
								}
							}
							
						//reset some variables for next use
							    
							    scount = 0;
							
								for (l=0; l <= STRING_LENGTH; l++){
									serial_in[l] = 0;
									temp[l] = '\0';
									data[l] = '\0';
								}
								
							cmd_state = COMMAND_IDLE;
							break;
							
					default :
							cmd_state = COMMAND_IDLE;                    //Default state
							break;
	
   } //switch case
  }	// stream status
		
		if (cmd == 'a' || cmd == 'A'){                                  //Set Attenuation
			//Function call here
			}
		if (cmd == 'f' || cmd == 'F'){                                  //Set Frequency
			
		    }
		if (cmd == 'v' || cmd == 'V'){                                  //Select Waveform
			
		    }
		if (cmd == 'b' || cmd == 'B'){                                  //Set Current ARB buffer to output
			
		    }
		if (cmd == 's' || cmd == 'S'){                                  //Return Status
			
		}
		else{
			opensda_uart_transmit_string("/E 1 \r\n");                  //unrecognized command
		}
		
		//reset parameter and data integers for next use
		parameter_value = 0;
		data_value = 0;
								
	
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

void UART0_IRQHandler(void){

	extern StreamBufferHandle_t UART_Rx_StreamHandle;
	char rx_data;
	

	/* Move a data byte from the receive data register to a stream buffer */
	/* The interrupt request is cleared by reading the UART0_D register */
	if(UART0_S1 & UART0_S1_RDRF_MASK){		/* Is it an Rx IRQ? */
		rx_data = UART0_D;
		xStreamBufferSendFromISR(UART_Rx_StreamHandle, &rx_data, 1,
				NULL);
		}
	return;
}
