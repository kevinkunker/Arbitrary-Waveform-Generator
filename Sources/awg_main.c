/*
 * awg_main.c
 *
 *  Created on: Apr 27, 2021
 *      Author: Owner
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


uint32_t timer_modulus = 2400000;                               /* bus clock cycles */
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
        int i = 0;
        
        UART0_C2 |= UART0_C2_RIE_MASK;          /* enable the UART receive interrupt */
        
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
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 2:
                                                                        dac_bit_shift = 1;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 4:
                                                                        dac_bit_shift = 2;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 8:
                                                                        dac_bit_shift = 3;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 16:
                                                                        dac_bit_shift = 4;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 32:
                                                                        dac_bit_shift = 5;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 64:
                                                                        dac_bit_shift = 6;
                                                                        opensda_uart_transmit_string("/OK /r/n");
                                                                        break;
                                                                case 128:
                                                                        dac_bit_shift = 7;
                                                                        opensda_uart_transmit_string("/OK /r/n");
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
                                                            	timer_modulus = 24000000;
                                                                set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK /r/n");
                                                                break;
                                                            case 10:
                                                                timer_modulus = 2400000;
                                                            	set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK /r/n");
                                                                break;
                                                            case 100:
                                                                timer_modulus = 240000;
                                                            	set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK /r/n");
                                                                break;
                                                            case 1000:
                                                            	timer_modulus = 24000; 
                                                                set_PIT_modulus(PIT_TIMER_0, timer_modulus);
                                                                opensda_uart_transmit_string("/OK /r/n");
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
                                                				memcpy(write_ptr, sine_data, DAC_WRITE_BUFFER_LEN);
                                                				opensda_uart_transmit_string("/OK /r/n");
                                                				break;
                                                			case 'T':
                                                			case 't':
                                                				memcpy(write_ptr, triangle_data, DAC_WRITE_BUFFER_LEN);
                                                				opensda_uart_transmit_string("/OK /r/n");
                                                				break;
                                                			case 'Q':
                                                			case 'q':
                                                				memset(write_ptr, 4095, DAC_WRITE_BUFFER_LEN/2);
                                                				memset(write_ptr + 50, 0, DAC_WRITE_BUFFER_LEN/2);
                                                				opensda_uart_transmit_string("/OK /r/n");
                                                				break;
                                                			case 'A':
                                                			case 'a':
                                                				memcpy(write_ptr, current_arb_buffer, DAC_WRITE_BUFFER_LEN);
                                                				opensda_uart_transmit_string("/OK /r/n");
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
                                                				opensda_uart_transmit_string("/OK /r/n");
                                                				break;
                                                			case 'B':
                                                			case 'b':
                                                				current_arb_buffer = ARB_buffer_b;
                                                				opensda_uart_transmit_string("/OK /r/n");
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
                                                        	char r_status[40] = "/S";
															switch(timer_modulus){
																case 24000000:
																	strcat(r_status, "1");
																	break;
																case 2400000:
																	strcat(r_status, "10");
																	break;
																case 240000:
																	strcat(r_status, "100");
																	break;
																case 24000:
																	strcat(r_status, "1000");
																	break;																
															}
                                                        }
                                                        break;
                                                case 'W':
                                                case 'w':
                                                		/* Arb Data Write */
                                                		break;
                                                case 'R':
                                                case 'r':
                                                		/* Arb Data Read */
                                                		break;
                                                default:
                                                        /* Error messages */
                                                	    /* Unrecognized command */
                                                        opensda_uart_transmit_string("/E 1");
                                                        break;
                                        }
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
        if(UART0_S1 & UART0_S1_RDRF_MASK){              /* Is it an Rx IRQ? */
                rx_data = UART0_D;
                xStreamBufferSendFromISR(UART_Rx_StreamHandle, &rx_data, 1,
                                NULL);
                }
        return;
}
