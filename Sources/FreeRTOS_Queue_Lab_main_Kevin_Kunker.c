/* FreeRTOS_Queue_Lab_main.c
 *  
 * History:
 * 	2021-3-9, sac,	created
 * 	2021-3-16, sac,	derived from FreeRTOS_Lab_1_main.c
 * 	
 * 	Overview:
 * 	---------
 * 	Uses two queues with four tasks. No timers or semaphores
 * 	
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "derivative.h"
#include "OpenSDA_UART.h"
#include "mcg.h"
#include "KL25Z_port.h"
#include "Shield_LEDs.h"
#include "Shield_pushbuttons.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

/* task timing */
#define SEND_MESSAGE_A_DELAY 1000
#define SEND_MESSAGE_B_DELAY 1333

#define TRANSMIT_QUEUE_LENGTH	10
#define TRANSMIT_QUEUE_SIZE		40
#define BUTTON_QUEUE_LENGTH		2
#define BUTTON_MESSAGE_SIZE		20

/* Function prototypes */
void SendMessageA(void *pvParameters);
void SendMessageB(void *pvParameters);
void TransmitMessage(void *pvParameters);
void ButtonMessage(void *pvParameters);

/* queue handles */
QueueHandle_t TransmitMessageQueue;
QueueHandle_t ButtonMessageQueue;

int main(void)
{
	/************************************************************
	 * Set the main clock to 48 MHz 
	 ************************************************************/
	pll_init(8000000,0,1,4,24,1);
	
	/************************************************************
	 * Enable Port/GPIO peripheral clocks 
	 ************************************************************/
	enable_port_clock(PORTA_PERIPHERAL);
	enable_port_clock(PORTB_PERIPHERAL);
	enable_port_clock(PORTC_PERIPHERAL);
	
	/************************************************************
	 *  Startup the Shield Pushbuttons
	 ************************************************************/
	init_shield_buttons();

	/************************************************************
	 *  Startup the OpenSDA UART (UART0)
	 ************************************************************/
	enable_opensda_uart_clock();
	init_opensda_uart_pins();
	init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_57600,
						OPENSDA_UART_STOP_BITS_1, OPENSDA_UART_PARITY_OFF, OPENSDA_UART_PARITY_EVEN);
	
	/* send out a welcome message */
	opensda_uart_transmit_string("ELT3050 FreeRTOS Queue Lab\r\n");
	
	/************************************************************
	 * create the FreeRTOS tasks 
	 ************************************************************/
	xTaskCreate(SendMessageA,		/* Pointer to the function that implements the task. */
				"Send A",			/* A name for the task. This is to facilitate debugging only. */
				100, 				/* Stack depth */
				NULL, 				/* Task parameter, not used */
				1,					/* Task priority */
				NULL );				/* task handle */
	
	xTaskCreate(TransmitMessage,	/* Pointer to the function that implements the task. */
				"Transmit",			/* A name for the task. This is to facilitate debugging only. */
				100, 				/* Stack depth */
				NULL, 				/* Task parameter, not used */
				1,					/* Task priority */
				NULL );				/* task handle */
	
	xTaskCreate(SendMessageB,		/* Pointer to the function that implements the task. */
				"Send B",			/* A name for the task. This is to facilitate debugging only. */
				100, 				/* Stack depth */
				NULL, 				/* Task parameter, not used */
				1,					/* Task priority */
				NULL );				/* task handle */
		
	xTaskCreate(ButtonMessage,		/* Pointer to the function that implements the task. */
				"Button Message",	/* A name for the task. This is to facilitate debugging only. */
				100, 				/* Stack depth */
				NULL, 				/* Task parameter, not used */
				1,					/* Task priority */
				NULL );				/* task handle */

	/************************************************************
	 * create the queues 
	 ************************************************************/
	TransmitMessageQueue = xQueueCreate(10, 40);
	ButtonMessageQueue = xQueueCreate(10, 20);

	/************************************************************
	 * start the FreeRTOS scheduler. Should never return
	 ************************************************************/
	vTaskStartScheduler();	
	
	/************************************************************
	 * should never reach here unless something goes wrong 
	 ************************************************************/
	for(;;) {}

	return 0;
}

/************************************************************
 *  Task functions
 ************************************************************/

/* send message A */
void SendMessageA( void *pvParameters ){
	
	extern QueueHandle_t TransmitMessageQueue;
	
	/* create and initialize a 40 byte, character array */
	char str40_1[] = "This string is transmitted from TaskA\r\n";
	
	for( ;; ){
		xQueueSendToBack(TransmitMessageQueue, str40_1, 0);
		vTaskDelay(SEND_MESSAGE_A_DELAY);
	}
}

/* send message B */
void SendMessageB(void *pvParameters){
	
	extern QueueHandle_t TransmitMessageQueue;
	
	/* Create and initialize a 40 byte character array */
	char str40_2[] = "This string is transmitted from TaskB\r\n";
	
	for(;;){
		xQueueSendToBack(TransmitMessageQueue, str40_2, 0);
		vTaskDelay(SEND_MESSAGE_B_DELAY);
	}
}

/* Button message */
void ButtonMessage(void *pvParameters){
	
	extern QueueHandle_t TransmitMessageQueue;
	
	/* Create and initialize a 20 byte character array */
	char str20[] = "Button 4 message.\r\n";
	
	for(;;){
		if(get_shield_pb4() == 0)
			xQueueSendToBack(ButtonMessageQueue, str20, 0);
	}
}

/* more task functions here */

/* transmit from queues to a serial port */
void TransmitMessage( void *pvParameters ){
	
	extern QueueHandle_t TransmitMessageQueue;
	
	/* create a 40 byte, character array */
	char str40received[40];
	/* create a 20 byte, character array */
	char str20received[20];
	
	for( ;; ){
		if(xQueueReceive(TransmitMessageQueue, str40received, 0) == pdPASS)
			opensda_uart_transmit_string(str40received);
		if(xQueueReceive(ButtonMessageQueue, str20received, 0) == pdPASS)
			opensda_uart_transmit_string(str20received);
	}
}


