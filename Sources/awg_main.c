/*
 * awg_main.c
 *
 *  Created on: Apr 27, 2021
 *      Authors: Kevin Kunker, Josh Remillard
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "derivative.h"
#include "OpenSDA_UART.h"
#include "mcg.h"
#include "KL25Z_port.h"
#include "analog.h"
#include "KL25Z_NVIC.h"
#include "KL25Z_pit.h"
#include "KL25Z_gpio.h"
#include "int_to_string_and_string_to_int.h"

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
#define UART_STREAM_SIZE                20
#define UART_STREAM_TRIG_LEVEL  1
#define RX_MESSAGE_BUFFER_SIZE  20
#define UART0_IRQ_PRIORITY      3

#define DAC_WRITE_BUFFER_LEN 100
#define STRING_LENGTH 10 


uint32_t timer_modulus = 240000;                               /* bus clock cycles */
uint32_t dac_write_buffer_1[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t dac_write_buffer_2[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t ARB_buffer_a[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t ARB_buffer_b[DAC_WRITE_BUFFER_LEN] = {0};
uint32_t *write_ptr = dac_write_buffer_1;
uint32_t *read_ptr = dac_write_buffer_2;
uint32_t *current_arb_buffer = ARB_buffer_a;
uint32_t dac_bit_shift = 0;


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
        
        /* Enable PORTB GPIO MUX*/
        
        set_port_mux(PORTB_PERIPHERAL, 0, ALT1);
        
        /************************************************************
        * Enable PTB0 for sync output
        ************************************************************/
        init_gpio_pin(GPIOB_PERIPHERAL, 0, GPIO_OUT);
        set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 0);
        
        /************************************************************
        *  Startup the OpenSDA UART (UART0)
        ************************************************************/
        init_opensda_uart_pins();
        init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_9600,
                          OPENSDA_UART_STOP_BITS_1, OPENSDA_UART_PARITY_OFF, OPENSDA_UART_PARITY_EVEN);
        
        /* send out a welcome message */
        opensda_uart_transmit_string("ELT3050 AWG Final Project \r\n");

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
        init_PIT(PIT_TIMER_0, timer_modulus, PIT_INT_ON);
        start_PIT(PIT_TIMER_0);
        
        /************************************************************
        * command interface task
        ************************************************************/
        xTaskCreate(Command_Interface_Task,                                 /* Pointer to the function that implements the task. */
                                "Command Interface Task",                               /* Friendly name for debugging. */
                                100,                                                    /* Stack depth (may need to be increased) */
                                NULL,                                                   /* Task parameter, not used */
                                1,                                                              /* Task priority */
                                NULL);                                                  /* task handle, not used */
        
        /************************************************************
         * create semaphores, queues, streams, etc.
         ************************************************************/
         
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

enum CommandState {
        COMMAND_IDLE,
        COMMAND_START,
        COMMAND_END
};

#define CMD_BUF_LEN 20

void Command_Interface_Task(void *pvParameters){
        
        extern StreamBufferHandle_t UART_Rx_StreamHandle;
        BaseType_t StreamStatus;
        char rx_char, cmd_state;
        char command_buffer[CMD_BUF_LEN] = {'\0'};
        char waveform;
        int i = 0;
        int point, data;
        
        UART0_C2 |= UART0_C2_RIE_MASK;          /* enable the UART receive interrupt */
        
        /* Initialize with sine wave as default */
        memcpy(write_ptr, sine_data, sizeof dac_write_buffer_1);
		/* Swap Buffers */
		if(write_ptr == dac_write_buffer_1){
			write_ptr = dac_write_buffer_2;
			read_ptr = dac_write_buffer_1;
		} else {
			write_ptr = dac_write_buffer_1;
			read_ptr = dac_write_buffer_2;
		}
		waveform = 'S';
        cmd_state = COMMAND_IDLE;
        
        for(;;){
                /* receive a character from the stream buffer with max delay, data in rx_char */
                StreamStatus = xStreamBufferReceive(UART_Rx_StreamHandle, &rx_char, 1, 0);
                
                if(StreamStatus != 0){
                        
                        /* check for a "/" to start command processing. Throw out anything else */
                        switch(cmd_state){
                                case COMMAND_IDLE:
									/* check for "/" */
									if(rx_char == '/'){
											cmd_state = COMMAND_START;
									}
									break;
                                case COMMAND_START:
                                        command_buffer[i++] = rx_char;
                                        if(rx_char == '\n' || rx_char == '\r' || i == CMD_BUF_LEN){
                                                cmd_state = COMMAND_END, i = 0;
                                        }
                                        break;
                                case COMMAND_END:
										for(i = 0; i < CMD_BUF_LEN - 1; i++){
											if(command_buffer[i++] == ' '){
												if(command_buffer[i] == ' '){
													/* Framing error */
													opensda_uart_transmit_string("/E 4");
												}
											}
										}
                                		
                                        switch(*strtok(command_buffer, " ")){
                                                case 'A':
                                                case 'a':
                                                        /* Set Attenuation */
                                                        switch(ascii_to_uint32(strtok(NULL, " "))){
                                                                case 1:
                                                                        dac_bit_shift = 0;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 2:
                                                                        dac_bit_shift = 1;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 4:
                                                                        dac_bit_shift = 2;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 8:
                                                                        dac_bit_shift = 3;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 16:
                                                                        dac_bit_shift = 4;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 32:
                                                                        dac_bit_shift = 5;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 64:
                                                                        dac_bit_shift = 6;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                case 128:
                                                                        dac_bit_shift = 7;
                                                                        opensda_uart_transmit_string("/OK \r\n");
                                                                        break;
                                                                default:
                                                                		opensda_uart_transmit_string("/E 2");
                                                                		break;
                                                        }
                                                        break;
                                                case 'F':
                                                case 'f':
                                                        /* Set Frequency */
                                                        switch(ascii_to_uint32(strtok(NULL, " "))){
                                                            case 1:
                                                            	timer_modulus = 240000;
                                                                set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK \r\n");
                                                                break;
                                                            case 10:
                                                                timer_modulus = 24000;
                                                            	set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                            	opensda_uart_transmit_string("/OK \r\n");
                                                                break;
                                                            case 100:
                                                                timer_modulus = 2400;
                                                            	set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                            	opensda_uart_transmit_string("/OK \r\n");
                                                                break;
                                                            case 1000:
                                                            	timer_modulus = 240; 
                                                                set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK \r\n");
                                                                break;
                                                            default:
                                                            	opensda_uart_transmit_string("/E 2");
                                                            	break;
                                                        }
                                                        break;
                                                case 'V':
                                                case 'v':
                                                        /* Select Waveform */
                                                		switch(*strtok(NULL, " ")){
                                                			case 'S':
                                                			case 's':
                                                				memcpy(write_ptr, sine_data, sizeof dac_write_buffer_1);
                                                				/* Swap Buffers */
																if(write_ptr == dac_write_buffer_1){
																	write_ptr = dac_write_buffer_2;
																	read_ptr = dac_write_buffer_1;
																} else {
																	write_ptr = dac_write_buffer_1;
																	read_ptr = dac_write_buffer_2;
																}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				waveform = 'S';
                                                				break;
                                                			case 'T':
                                                			case 't':
                                                				memcpy(write_ptr, triangle_data, sizeof dac_write_buffer_1);
                                                				/* Swap Buffers */
																if(write_ptr == dac_write_buffer_1){
																	write_ptr = dac_write_buffer_2;
																	read_ptr = dac_write_buffer_1;
																} else {
																	write_ptr = dac_write_buffer_1;
																	read_ptr = dac_write_buffer_2;
																}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				waveform = 'T';
                                                				break;
                                                			case 'Q':
                                                			case 'q': {
                                                				int j;
                                                   				for(j = 0; j < 50; j++){
																	write_ptr[j] = 4095;
																}
																for(j = 50; j < 100; j++){
																	write_ptr[j] = 0;
																}
                                                				/* Swap Buffers */
																if(write_ptr == dac_write_buffer_1){
																	write_ptr = dac_write_buffer_2;
																	read_ptr = dac_write_buffer_1;
																} else {
																	write_ptr = dac_write_buffer_1;
																	read_ptr = dac_write_buffer_2;
																}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				waveform = 'Q';
                                                				break;
                                                			}
                                                			case 'A':
                                                			case 'a':
                                                				memcpy(write_ptr, current_arb_buffer, sizeof dac_write_buffer_1);
                                                				/* Swap Buffers */
																if(write_ptr == dac_write_buffer_1){
																	write_ptr = dac_write_buffer_2;
																	read_ptr = dac_write_buffer_1;
																} else {
																	write_ptr = dac_write_buffer_1;
																	read_ptr = dac_write_buffer_2;
																}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				waveform = 'A';
                                                				break;
                                                			default:
                                                			    opensda_uart_transmit_string("/E 2");
                                                			    break;
                                                			
                                                		}
                                                		
                                                        break;
                                                case 'B':
                                                case 'b':
                                                        /* Set Current ARB Buffer */
                                                		switch(*strtok(NULL, " ")){
                                                			case 'A':
                                                			case 'a':
                                                				current_arb_buffer = ARB_buffer_a;
                                                				if(waveform == 'A'){
                                                					memcpy(write_ptr, current_arb_buffer, sizeof dac_write_buffer_1);
																	/* Swap Buffers */
																	if(write_ptr == dac_write_buffer_1){
																		write_ptr = dac_write_buffer_2;
																		read_ptr = dac_write_buffer_1;
																	} else {
																		write_ptr = dac_write_buffer_1;
																		read_ptr = dac_write_buffer_2;
																	}
                                                				}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				break;
                                                			case 'B':
                                                			case 'b':
                                                				current_arb_buffer = ARB_buffer_b;
                                                				if(waveform == 'A'){
																	memcpy(write_ptr, current_arb_buffer, sizeof dac_write_buffer_1);
																	/* Swap Buffers */
																	if(write_ptr == dac_write_buffer_1){
																		write_ptr = dac_write_buffer_2;
																		read_ptr = dac_write_buffer_1;
																	} else {
																		write_ptr = dac_write_buffer_1;
																		read_ptr = dac_write_buffer_2;
																	}
																}
                                                				opensda_uart_transmit_string("/OK \r\n");
                                                				break;
                                                			default:
                                                				opensda_uart_transmit_string("/E 2");
                                                			    break;
                                                		}
                                                        break;
                                                case 'S':
                                                case 's':
                                                        /* Return Status */
                                                        {
                                                        	char r_status[40] = {'\0'};
                                                        	strcat(r_status, "/S ");
															switch(timer_modulus){
																case 240000:
																	strcat(r_status, "1");
																	break;
																case 24000:
																	strcat(r_status, "10");
																	break;
																case 2400:
																	strcat(r_status, "100");
																	break;
																case 240:
																	strcat(r_status, "1000");
																	break;																
															}
															strcat(r_status, " ");
															switch (dac_bit_shift){
															    case 0:
															    	strcat(r_status, "1");
															    	break;
															    case 1:
															    	strcat(r_status, "2");
															    	break;
															    case 2:
															    	strcat(r_status, "4");
															    	break;
															    case 3:
															    	strcat(r_status, "8");
															    	break;
															    case 4:
															    	strcat(r_status, "16");
															    	break;
															    case 5:
															    	strcat(r_status, "32");
															    	break;
															    case 6:
															    	strcat(r_status, "64");
															    	break;
															    case 7:
															    	strcat(r_status, "128");
															    	break;
															}
															strcat(r_status, " ");
															switch (waveform){
															    case 'S':
															    	strcat(r_status, "S");
															    	break;
															    case 'T':
															    	strcat(r_status, "T");
															    	break;
															    case 'Q':
															    	strcat(r_status, "Q");
															    	break;
															    case 'A':
															    	strcat(r_status, "A");
															    	break;
															}
															strcat(r_status, " ");
															if((current_arb_buffer == ARB_buffer_a)){
																	strcat(r_status, "A");
															}
															else{	
																	strcat(r_status, "B");
																	
															}
															strcat(r_status, "\r\n");
															opensda_uart_transmit_string(r_status);
                                                        }
                                                        break;
                                                case 'W':
                                                case 'w':
                                                		/* Arb Data Write */
                                                        {
                                                		point = (ascii_to_uint32(strtok(NULL, " ")));
                                                		data = (ascii_to_uint32(strtok(NULL, " ")));
                                                		if (data>4095){
                                                			opensda_uart_transmit_string("/E 3");
                                                		}
                                                  		if (data<4096 && (current_arb_buffer == ARB_buffer_a)){
                                                			ARB_buffer_b[point] = data;
                                                			opensda_uart_transmit_string("/OK \r\n");
                                                		}
                                                  		if (data<4096 && (current_arb_buffer == ARB_buffer_b)){
                                                  			ARB_buffer_a[point] = data;
                                                  			opensda_uart_transmit_string("/OK \r\n");
                                                  		}
                                                        }
                                                  		break;
                                                case 'R':
                                                case 'r':
                                                		/* Arb Data Read */
                                                        {
                                                	    char ARB_data_read[15] = {'\0'};
                                                	    char point_value_string[1];
                                                	    char data_value_string[3];
                                                	    
                                                	    strcat(ARB_data_read, "/R ");
                                                		
                                                	    point = (ascii_to_uint32(strtok(NULL, " ")));
                                            			
                                              			if (point>99){
                                                			opensda_uart_transmit_string("/E 3");
                                                		}
                                              			else{
                                              				uint32_to_ascii(point,point_value_string);
                                              				strcat(ARB_data_read, point_value_string);
                                              				strcat(ARB_data_read, " ");
                                              			}
                                
                                                  		if (point<100 && (current_arb_buffer == ARB_buffer_a)){
                                                   			uint32_to_ascii(ARB_buffer_b[point], data_value_string);
                                                   			strcat(ARB_data_read, data_value_string);
                                                   			strcat(ARB_data_read, "\r\n");
                                                   			opensda_uart_transmit_string(ARB_data_read);
                                                   		}
                                                  		if (point<100 && (current_arb_buffer == ARB_buffer_b)){
                                                            uint32_to_ascii(ARB_buffer_a[point], data_value_string);
                                                  		    strcat(ARB_data_read, data_value_string);
                                                  		    strcat(ARB_data_read, "\r\n");
                                                  		    opensda_uart_transmit_string(ARB_data_read);
                                                        }
                                                        }

                                                  		break;
                                                default:
                                                        /* Error messages */
                                                	    /* Unrecognized command */
                                                        opensda_uart_transmit_string("/E 1");
                                                        break;
                                        }
                                        
                                        /* Reset command buffer */
                                        i = 0;
                                        memset(command_buffer, 0, CMD_BUF_LEN);
                                        
                                        cmd_state = COMMAND_IDLE;
                                        break;
                                default:
                                        cmd_state = COMMAND_IDLE;
                                        break;
                        }
                }
        }
}

void PIT_IRQHandler(){
        
        static int i = 0;
        
        clear_PIT_int_flag(PIT_TIMER_0);        /* acknowledge the IRQ in the timer*/
        
        set_dac_output(read_ptr[i] >> dac_bit_shift);
        
        
        switch(i){
			case 0:
				set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 1);
				i++;
				break;
			case DAC_WRITE_BUFFER_LEN/2:
				set_gpio_pin_level(GPIOB_PERIPHERAL, 0, 0);
				i++;
				break;
			case DAC_WRITE_BUFFER_LEN-1:
				i = 0;
				break;
			default:
				i++;
				break;
		}
        return;
}

void UART0_IRQHandler(void){

        extern StreamBufferHandle_t UART_Rx_StreamHandle;
        char rx_data;
        

        /* Move a data byte from the receive data register to a stream buffer */
        /* The interrupt request is cleared by reading the UART0_D register */
        if(UART0_S1 & UART0_S1_RDRF_MASK){              /* Is it an Rx IRQ? */
                rx_data = UART0_D;
                xStreamBufferSendFromISR(UART_Rx_StreamHandle, &rx_data, 1,
                                NULL);
                }
        return;
}
