/* FreeRTOS_Semaphore_Lab_main.c
 * 
 * Two tasks send messages to a serial port. They use a mutex for access to the serial port.
 * The tasks are timed to send messages by a software time and a binry semaphore
 *  
 * History:
 * 	2021-03-23, sac, created
 * 	                
 * 	Stack Overflow Hook Issue
 * 	-------------------------
 * 	Note! None of the vApplicationStackOverflowHook() prototypes in the documentation work.
 * 	There are two. These is a type mismatch with the FreeRTOS code. 
 * 	I think the cause is in the bowels of the TCB types and gcc. One of the mismatches is
 * 	between "char" and "signed char". Here is a prototype which works. Magic...
 * 	                
 * 	     void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
 * 	     
 * 	Also, consider using portCHAR as the char type. This is in portmacro.h
 * 	     
 * 	Caution! Type 2 stack overflow checking uses bytes not stack words!. So, the 
 * 	level is 16 or 20 BYTES (4 or 5 32 bit stack words)!
 * 	
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

/* Program configuration */
#define USE_STACK_OVERFLOW_CHECKING

/* task timing */
#define SEND_MESSAGE_PERIOD	 1000

/* Function prototypes */
void SendMessageA(void *pvParameters);
void SendMessageB(void *pvParameters);
void CollectHighWaterMarks(void *pvParameters);
void SendMessageATimerCallback(TimerHandle_t xTimer);
void SendMessageBTimerCallback(TimerHandle_t xTimer);
#ifdef USE_STACK_OVERFLOW_CHECKING
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

/* handles */
TaskHandle_t SendMessageA_Handle;
TaskHandle_t SendMessageB_Handle;
TaskHandle_t CollectHighWaterMarks_Handle;

TimerHandle_t SendMessageATimer;
TimerHandle_t SendMessageBTimer;
SemaphoreHandle_t SendMessageASemaphore;
SemaphoreHandle_t SendMessageBSemaphore;
SemaphoreHandle_t SerialPortMutex;

/* stack checking */
#define NUM_STACK_CHECKS 3
UBaseType_t	TaskHighWaterMarks[NUM_STACK_CHECKS]; 

int main(void)
{
	extern TaskHandle_t SendMessageA_Handle;
	extern TaskHandle_t SendMessageB_Handle;
	extern TaskHandle_t CollectHighWaterMarks_Handle;
	extern TimerHandle_t SendMessageATimer;
	extern TimerHandle_t SendMessageBTimer;
	extern SemaphoreHandle_t SendMessageASemaphore;
	extern SemaphoreHandle_t SendMessageBSemaphore;
	extern SemaphoreHandle_t SerialPortMutex;

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
	 *  Startup the OpenSDA UART (UART0)
	 ************************************************************/
	enable_opensda_uart_clock();
	init_opensda_uart_pins();
	init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_57600,
						OPENSDA_UART_STOP_BITS_1, OPENSDA_UART_PARITY_OFF, OPENSDA_UART_PARITY_EVEN);
	
	/* send out a welcome message */
	opensda_uart_transmit_string("ELT3050 FreeRTOS Semaphore and Stack Lab\r\n");
	
	/************************************************************
	 * create the tasks 
	 ************************************************************/
	
	/* Minimum stack size needed to not get a stack overflow: 37 */
	xTaskCreate(SendMessageA,		/* Pointer to the function that implements the task. */
				"Send A",			/* A name for the task. This is to facilitate debugging only. */
				87, 				/* Stack depth in 32 bit words */
				NULL, 				/* Task parameter, not used */
				1,					/* Task priority */
				&SendMessageA_Handle );				/* task handle */
	
	/* create the other tasks */
	xTaskCreate(SendMessageB,
				"Send B",
				70,
				NULL,
				1,
				&SendMessageB_Handle);
	
	xTaskCreate(CollectHighWaterMarks,
				"High water",
				87,
				NULL,
				1,
				&CollectHighWaterMarks_Handle);
	
	/************************************************************
	 * create FreeRTOS timers 
	 ************************************************************/
	SendMessageATimer = xTimerCreate(
						"Message A Timer",
						SEND_MESSAGE_PERIOD,
						pdTRUE,				/* repetitive timer */
						0,					/* timer ID not used */
						SendMessageATimerCallback);
	SendMessageBTimer = xTimerCreate(
						"Message B Timer",
						SEND_MESSAGE_PERIOD,
						pdTRUE,
						0,
						SendMessageBTimerCallback);
	
	xTimerStart(SendMessageATimer, 0);
	xTimerStart(SendMessageBTimer, 0);
	
	/************************************************************
	 * create semaphores
	 ************************************************************/
	SendMessageASemaphore = xSemaphoreCreateBinary();
	SendMessageBSemaphore = xSemaphoreCreateBinary();
	SerialPortMutex = xSemaphoreCreateMutex();

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

/************************************************************
 *  Task functions
 ************************************************************/

/* stack checking */
void CollectHighWaterMarks( void *pvParameters ){

	extern UBaseType_t	TaskHighWaterMarks[NUM_STACK_CHECKS]; 

	for( ;; ){
		
		/* get the task highwater marks here */
		TaskHighWaterMarks[0] = uxTaskGetStackHighWaterMark(SendMessageA_Handle);
		TaskHighWaterMarks[1] = uxTaskGetStackHighWaterMark(SendMessageB_Handle);
		TaskHighWaterMarks[2] = uxTaskGetStackHighWaterMark(CollectHighWaterMarks_Handle);
	}
}

/* send message A */
void SendMessageA( void *pvParameters ){
	
	extern SemaphoreHandle_t SendMessageASemaphore;
	extern SemaphoreHandle_t SerialPortMutex;

	static char message_a[] = "Task A - I got the serial port!!!!!!!\r\n";
	
	for( ;; ){

		/* wait on the binary semaphore here */
		xSemaphoreTake(SendMessageASemaphore, portMAX_DELAY);
		
		/* Try to take the serial port mutex here,
		 * If successful, send the string
		 */
		if(xSemaphoreTake(SerialPortMutex, portMAX_DELAY)){
			opensda_uart_transmit_string(message_a);
			xSemaphoreGive(SerialPortMutex);
		}
	}
}

/* send message B */
void SendMessageB( void *pvParameters ){
	
	extern SemaphoreHandle_t SendMessageBSemaphore;
	extern SemaphoreHandle_t SerialPortMutex;

	static char message_b[] = "Task B - My turn\r\n";

	xTimerStart(SendMessageATimer, 0);
	
	for( ;; ){
		
		/* same logic as the other task */
		xSemaphoreTake(SendMessageBSemaphore, portMAX_DELAY);
		
		if(xSemaphoreTake(SerialPortMutex, portMAX_DELAY)){
			opensda_uart_transmit_string(message_b);
			xSemaphoreGive(SerialPortMutex);
		}
	}
}

/************************************************************
 *  callbacks and hooks
 ************************************************************/

/* timer callback */
void SendMessageATimerCallback(TimerHandle_t xTimer){

	extern SemaphoreHandle_t SendMessageASemaphore;
	
	/* give the semaphores to release the message tasks */
	xSemaphoreGive(SendMessageASemaphore);
	
	return;
}

void SendMessageBTimerCallback(TimerHandle_t xTimer){
	
	extern SemaphoreHandle_t SendMessageBSemaphore;
	
	xSemaphoreGive(SendMessageBSemaphore);
	
	return;
}

/*  stack overflow hook
 * 
 * 	Stack Overflow Hook Issue
 * 	-------------------------
 * 	Note! None of the vApplicationStackOverflowHook() prototypes in the documentation work.
 * 	These is a type mismatch with the FreeRTOS code. I think the cause is in the bowels of
 * 	the TCB types and gcc. One of the mismatches is	between "char" and "signed char".
 * 	Here is a prototype which works. Magic...
 * 	                
 */
#ifdef USE_STACK_OVERFLOW_CHECKING
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	__asm ("bkpt  \n");
	return;
}
#endif

