/* FreeRTOS_ADC_Experiment_main.c
 *
 * 16 bit, ADC acquisitions are done with a hardware timer triggering the conversions.
 * An ADC interrupt handler reads the data from the ADC when a conversion is complete.
 * No DMA is used to collect the data.
 * 
 * Acquisition options include averaging conversions in the ADC and using the window
 * comparison feature in the ADC to gate the acquisition.
 * 
 * Data is sent out the OpenSDA serial port as a scrolling bar graph display. The serial port
 * receives siple, CLI commands to select acquisition options.
 *
 * 2021-04-01, sac, created
 * 
 * Terminal baud rate is 115200
 *  
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

/* program configuration, put #defines here */
#define ADC_AVERAGING
#define ADC_COMPARE
#define ADC_INPUT_TRIMPOT

/* serial parameters */
#define UART_STREAM_SIZE		20
#define UART_STREAM_TRIG_LEVEL	1
#define RX_MESSAGE_BUFFER_SIZE	20
#define UART0_IRQ_PRIORITY	2

/* ADC parameters */
#define HARD_TIMER_MODULUS   2400000   /* bus clock cycles */

#define ADC_QUEUE_LENGTH 10
#define ADC_QUEUE_SIZE   4

/* Define the ADC compare levels here, CV1 and CV2 register */
#define ADC_COMPARE_HIGH_LEVEL	39719	//TBD according to the spec
#define ADC_COMPARE_LOW_LEVEL	19859	//TBD according to the spec

/* Function prototypes */
void TestDAC( void *pvParameters );
void AcquireADC(void *pvParameters);
void ADC0_IRQHandler(void);
void Commands(void *pvParameters);
	
/* handles */
QueueHandle_t ADCQueue;
StreamBufferHandle_t  UART_Rx_StreamHandle;
SemaphoreHandle_t SerialPortMutex;

/* Misc */
uint32_t max = 0, min = 0, peak_to_peak = 0, latest = 0;
char rx_message[RX_MESSAGE_BUFFER_SIZE];

int main(void)
{
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
	enable_adc_clock();
	enable_opensda_uart_clock();
	enable_pit_clock();
	enable_dac_clock();

	/************************************************************
	 *  Startup the OpenSDA UART (UART0)
	 ************************************************************/
	init_opensda_uart_pins();
	init_opensda_uart(OPENSDA_UART_BAUD_CLOCK_MCGPLLCLKDIV2, OPENSDA_UART_BAUD_115200,
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

	init_shield_adc_pins();
	init_adc(ADC_16BIT_RESOLUTION);
	
	enable_bandgap_reference();
	
	/* select PIT1 as the ADC hardware trigger */
	SIM_SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK | SIM_SOPT7_ADC0TRGSEL(5);
	
	/* ADC IRQ settings */
	NVIC_DisableIRQ(ADC0_IRQn);
	NVIC_SetPriority(ADC0_IRQn, 3);
	NVIC_EnableIRQ(ADC0_IRQn);
	
	/* hardware timer setup */
	init_PIT(PIT_TIMER_1, HARD_TIMER_MODULUS, PIT_INT_OFF);
	
	/************************************************************
	 * create the tasks 
	 ************************************************************/
	xTaskCreate(AcquireADC,				/* Pointer to the function that implements the task. */
				"ADC Task",				/* A name for the task. This is to facilitate debugging only. */
				100, 					/* Stack depth in 32 bit words */
				NULL, 					/* Task parameter, not used */
				1,						/* Task priority */
				NULL);					/* task handle */
	
	/* Create the "Commands" task */
	xTaskCreate(Commands,
				"Commands Task",
				100,
				NULL,
				1,
				NULL);
	
	/************************************************************
	 * create semaphores, queues, streams, etc.
	 ************************************************************/
	/* Create the ADC data queue */
	ADCQueue = xQueueCreate(ADC_QUEUE_LENGTH, ADC_QUEUE_SIZE);
	
	/* Create the UARTO data stream buffer */
	UART_Rx_StreamHandle = xStreamBufferCreate(UART_STREAM_SIZE, UART_STREAM_TRIG_LEVEL);
	
	/* Create the Serial Port Transmit Mutex */
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
 *  Tasks
 ************************************************************/


/* acquire and transmit ADC data */
void AcquireADC( void *pvParameters ){
	
	extern QueueHandle_t ADCQueue;
	extern uint32_t max, min, peak_to_peak, latest;
	uint32_t data, i, display;
	char message[80] = "????????????????\r\n";

#ifdef ADC_AVERAGING //TBD
	/* setup for ADC averaging */
	ADC0_SC3 = (ADC0_SC3 & ~(ADC_SC3_AVGS_MASK | ADC_SC3_AVGE_MASK)) | 0x3;  /* average 32 */
	ADC0_SC2 &=	~(ADC_SC2_ACFGT_MASK);  /* out of window compare */
	ADC0_SC2 |=	ADC_SC2_ACREN_MASK;     /*  "  "    "      "     */
	ADC0_SC3 |=	ADC_SC3_AVGE_MASK;
#endif

#ifdef ADC_COMPARE
	/* setup for ADC window compare */
	ADC0_CV2 = ADC_COMPARE_HIGH_LEVEL;
	ADC0_CV1 = ADC_COMPARE_LOW_LEVEL;
	ADC0_SC2 |= ADC_SC2_ACFE_MASK;
#endif

	/* setup for ADC data measurements */
	max = 0;
	min = 65535;
	
	/* select an analog input */
#ifdef ADC_INPUT_BG
	setup_adc(KL25Z_BG_REFERENCE, ADC_INPUT_SINGLE, 1, 1, 0); /* interrupt, trigger, no dma */
#endif
#ifdef ADC_INPUT_TEMP
	setup_adc(KL25Z_TEMPERATURE, ADC_INPUT_SINGLE, 1, 1, 0); /* interrupt, trigger, no dma */
#endif
#ifdef ADC_INPUT_TRIMPOT
	setup_adc(SHIELD_TRIMPOT, ADC_INPUT_SINGLE_B, 1, 1, 0); /* interrupt, trigger, no dma */
#endif

	/* start the PIT1 timer here which starts the ADC */
	start_PIT(PIT_TIMER_1);

	for( ;; ){

		/* receive ADC data from the queue into the "data" variable */
		xQueueReceive(ADCQueue, &data, portMAX_DELAY);
		
		/* produce some measurements to see with the debugger */
		if(data > max) max = data;
		if(data < min) min = data;
		peak_to_peak = max - min;
		latest = data;
		
		/* send scaled ADC data to the DAC for monitoring */
		set_dac_output(data >> 4);
		
		display = data >> 8;  /* scale to 8 bits */
		i = 0;		          /* build the string */
		if(display <= 255){   /* within range? */
			while (display >= 4){
				message[i++] = '_';
				display = display - 4;
			}
			message[i++] = '\r';
			message[i++] = '\n';
			message[i]   = '\0';
		}
		else {
			message[i++] = 'R';
			message[i++] = 'N';
			message[i++] = 'G';
			message[i++] = '\r';
			message[i++] = '\n';
			message[i]   = '\0';
		};
		
		/* send the string to the serial port */
		/* take/give the serial port mutex if command response is active */
		xSemaphoreTake(SerialPortMutex, portMAX_DELAY);
		opensda_uart_transmit_string(message);
		xSemaphoreGive(SerialPortMutex);
		
	}; /* end forever */
}

/* UART receive task
 * This task blocks until a character is the input stream buffer.
 * This frees up the processor until the UART ISR receives a character
 */

/* command state machine states */
enum CommandStates {
	COMMAND_IDLE,
	COMMAND_START
};

/* serial port command task */
void Commands(void *pvParameters){

	extern StreamBufferHandle_t UART_Rx_StreamHandle;
	BaseType_t StreamStatus;
	char rx_char, cmd_state;
	
	UART0_C2 |= UART0_C2_RIE_MASK;		/* enable the UART receive interrupt */

	cmd_state = COMMAND_IDLE;
	for( ;; ){
		/* receive a character from the stream buffer with max delay, data in rx_char */
		StreamStatus = xStreamBufferReceive(UART_Rx_StreamHandle, &rx_char, 1, 0);
		
		if (StreamStatus != 0){
			
			/* check for a "/" to start command processing. Throw out anything else */
			switch(cmd_state){
				case COMMAND_IDLE :
					/* check for "/" */
					if(rx_char == '/'){
						cmd_state = COMMAND_START;
					}
					break;
				case COMMAND_START :
					/* check the next rx_char */
					switch(rx_char){
						case 'T':
						case 't':
							stop_PIT(PIT_TIMER_1);
							setup_adc(KL25Z_TEMPERATURE, ADC_INPUT_SINGLE, 1, 1, 0);
							start_PIT(PIT_TIMER_1);
							xSemaphoreTake(SerialPortMutex, portMAX_DELAY);
							opensda_uart_transmit_string("Temperature Selected\r\n");
							xSemaphoreGive(SerialPortMutex);
							cmd_state = COMMAND_IDLE;
							break;
						case 'B':
						case 'b':
							stop_PIT(PIT_TIMER_1);
							setup_adc(KL25Z_BG_REFERENCE, ADC_INPUT_SINGLE, 1, 1, 0);
							start_PIT(PIT_TIMER_1);
							xSemaphoreTake(SerialPortMutex, portMAX_DELAY);
							opensda_uart_transmit_string("Reference Selected\r\n");
							xSemaphoreGive(SerialPortMutex);
							cmd_state = COMMAND_IDLE;
							break;
						case 'P':
						case 'p':
							stop_PIT(PIT_TIMER_1);
							setup_adc(SHIELD_TRIMPOT, ADC_INPUT_SINGLE_B, 1, 1, 0);
							start_PIT(PIT_TIMER_1);
							xSemaphoreTake(SerialPortMutex, portMAX_DELAY);
							opensda_uart_transmit_string("Potentiometer Selected\r\n");
							xSemaphoreGive(SerialPortMutex);
							cmd_state = COMMAND_IDLE;
							break;
					}
					break;
				default :
					break;
			} /* switch cmd_state */
		} /* if StreamStatus */
	} /* forever */
}


/************************************************************
 *  interrupt handlers
 ************************************************************/
void ADC0_IRQHandler(void){

	extern QueueHandle_t ADCQueue;
	uint32_t adc_data;

	/* get the data from the ADC */
	adc_data = ADC0_RA;
	
	/* send the data to the queue. Remember! Use the "FromISR" form */
	xQueueSendToBackFromISR(ADCQueue, &adc_data, NULL);

	return;
}

/*  UART0 IRQ Task_UART_Tx_HandleHandler */
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
