
/* FreeRTOS_Lab_1_main.c
 *  
 * History:
 * 	2021-3-6, sac,	created *
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

/* Some Shield LED macros */
#define LED1_ON  (GPIOC_PCOR = 0x1 << LED1_PIN)
#define LED1_OFF (GPIOC_PSOR = 0x1 << LED1_PIN)
#define LED2_ON  (GPIOC_PCOR = 0x1 << LED2_PIN)
#define LED2_OFF (GPIOC_PSOR = 0x1 << LED2_PIN)
#define LED3_ON  (GPIOC_PCOR = 0x1 << LED3_PIN)
#define LED3_OFF (GPIOC_PSOR = 0x1 << LED3_PIN)
#define LED4_ON  (GPIOC_PCOR = 0x1 << LED4_PIN)
#define LED4_OFF (GPIOC_PSOR = 0x1 << LED4_PIN)

/* task timing */
#define TASKA_TICK_DELAY	333
#define TASKB_TICK_DELAY	1
#define TASKC_TICK_DELAY	10

/* Function prototypes */
void TaskA(void *pvParameters);
void TaskB(void *pvParameters);
void TaskC(void *pvParameters);

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
	 *  Startup the Shield LEDs and Pushbuttons
	 ************************************************************/
	init_shield_buttons();
	init_shield_leds();

	/************************************************************
	 *  Startup the OpenSDA UART (UART0)
	 ************************************************************/
	enable_opensda_uart_clock();
	init_opensda_uart_pins();
	init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_57600,
						OPENSDA_UART_STOP_BITS_1, OPENSDA_UART_PARITY_OFF, OPENSDA_UART_PARITY_EVEN);
	
	/* send out a welcome message */
	opensda_uart_transmit_string("ELT3050 FreeRTOS Lab 1\r\n");
	
	/************************************************************
	 * create three FreeRTOS tasks and start the Scheduler 
	 ************************************************************/

	/* see the FreeRTOS documentation and lab procedure for parameters */
	xTaskCreate(TaskA, "Task A", 70, NULL, 1, NULL);
	xTaskCreate(TaskB, "Task B", 70, NULL, 1, NULL);
	xTaskCreate(TaskC, "Task C", 70, NULL, 1, NULL);


	/* start the FreeRTOS scheduler. Should never return */
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

enum TaskALEDPattern {
	BOTH_OFF, FIRST_ON, SECOND_ON, BOTH_ON
};

/* blink some LEDS */
void TaskA( void *pvParameters ){
	
	for( ;; ){
		static enum TaskALEDPattern ledPattern = BOTH_OFF;
		
		switch(ledPattern){
			case BOTH_OFF:
				LED1_OFF;
				LED2_OFF;
				ledPattern = FIRST_ON;
				break;
			case FIRST_ON:
				LED1_ON;
				LED2_OFF;
				ledPattern = SECOND_ON;
				break;
			case SECOND_ON:
				LED1_OFF;
				LED2_ON;
				ledPattern = BOTH_ON;
				break;
			case BOTH_ON:
				LED1_ON;
				LED2_ON;
				ledPattern = BOTH_OFF;
				break;
			default:
				ledPattern = BOTH_OFF;
				break;
		}
		vTaskDelay(TASKA_TICK_DELAY);
	}	/* end forever loop */
}

/* write to a serial port */
void TaskB( void *pvParameters ){

	for( ;; ){
		opensda_uart_transmit_string("This is a test message that I hope I can receive\r\n");
		vTaskDelay(TASKB_TICK_DELAY);
	}
}

/* monitor a pushbutton and light an LED if pushed */
void TaskC( void *pvParameters ){

	for( ;; ){
		
		if(get_shield_pb4() == 0)  /* button is active low */
			LED4_ON;
		else
			LED4_OFF;
		
		vTaskDelay(TASKC_TICK_DELAY);
	}
}

/************************************************************
 *  Callback Functions (hooks)
 *  
 *  Enable hooks with the macros in FreeRTOSConfig.h
 *  #define configUSE_IDLE_HOOK		1
 *  #define configUSE_TICK_HOOK		1
 *  
 ************************************************************/

//void vApplicationIdleHook(void){
//}

//void vApplicationTickHook(void){
//};
