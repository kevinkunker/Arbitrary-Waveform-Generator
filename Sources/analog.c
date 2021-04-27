/*
 * analog.c
 * 
 * Support for analog functions such as the ADC and DAC
 * 
 *  Created on: Oct 10, 2020
 *      Author: sac
 *	2020-10-15, sac, added CMP stuff
 *	2021-03-26, sac, added enable_adc_conversions() for triggered conversions with options for interrupt
 *	                 and dma requests
 */

#include "derivative.h"
#include "KL25Z_port.h"
#include "analog.h"

/* enable the bus clock in the ADC */
void enable_adc_clock(void){
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	return;
}

/* ADC initialization
 * 
 * The conversion clock is the bus clock divided by 16. (24/16 = 1.5 MHz for core clock of 48 MHz)
 *   
 * Parameters:
 * 	resolution -	8 bits: 0
 * 					10 bits: 2
 * 					12 bits: 1
 * 					16 bits: 3
 */ 

void init_adc(uint32_t resolution){    
 
	ADC0_CFG1 = ((resolution << ADC_CFG1_MODE_SHIFT) & ADC_CFG1_MODE_MASK) | (ADC_CFG1_ADICLK_MASK & 0x1)
				| (0x3 << ADC_CFG1_ADIV_SHIFT);
	ADC0_SC2 = 0;
	return;
}


/* performs a single, software triggered adc conversion
 *	parameters
 *		channel - adc channel number
 *		input_type - single ended, single ended A, single ended B or differential 
 */
uint32_t convert_adc(uint32_t channel, uint32_t input_type){    

	switch (input_type){
		case ADC_INPUT_SINGLE : 
		case ADC_INPUT_SINGLE_A : 
			ADC0_CFG2 = 0;
			ADC0_SC1A = channel & ADC_SC1_ADCH_MASK;
			break;
		case ADC_INPUT_SINGLE_B :
			ADC0_CFG2 = 1 << ADC_CFG2_MUXSEL_SHIFT;
			ADC0_SC1A = channel & ADC_SC1_ADCH_MASK;
			break;
		case ADC_INPUT_DIFFERENTIAL :
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | ADC_SC1_DIFF_MASK;
			break;
	};
	
	while(!(ADC_SC1_COCO_MASK & ADC0_SC1A));
	return ADC0_RA;
};

/* same as convert_adc() but does not return the result of the conversion
 * Software trigger with interrupt request enabled.
 * For use with an ISR which gets the data
 */
void convert_adc_with_irq(uint32_t channel, uint32_t input_type){    

	switch (input_type){
		case ADC_INPUT_SINGLE : 
		case ADC_INPUT_SINGLE_A : 
			ADC0_CFG2 = 0;
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | ADC_SC1_AIEN_MASK;
			break;
		case ADC_INPUT_SINGLE_B :
			ADC0_CFG2 = 1 << ADC_CFG2_MUXSEL_SHIFT;
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | ADC_SC1_AIEN_MASK;
			break;
		case ADC_INPUT_DIFFERENTIAL :
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | ADC_SC1_DIFF_MASK | ADC_SC1_AIEN_MASK;
			break;
	};
	
	return;
};

/* Setups ADC for hardware trigger, interrupts or dma request
 * Software trigger with interrupt request enabled.
 * For use with an ISR which gets the data
 */
void setup_adc(uint32_t channel, uint32_t input_type,
		       uint32_t interrupt, uint32_t trigger,
		       uint32_t dma){
	
	uint32_t adc_register = 0;
	
	/* SC2 register */
	if(trigger) ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
	if(dma) ADC0_SC2 |= ADC_SC2_DMAEN_MASK;

	/* CFG2 register and SC1A register */
		if(interrupt) adc_register =  ADC_SC1_AIEN_MASK;
	switch (input_type){
		case ADC_INPUT_SINGLE : 
		case ADC_INPUT_SINGLE_A : 
			ADC0_CFG2 = 0;
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | adc_register;
			break;
		case ADC_INPUT_SINGLE_B :
			ADC0_CFG2 = 1 << ADC_CFG2_MUXSEL_SHIFT;
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | adc_register;
			break;
		case ADC_INPUT_DIFFERENTIAL :
			ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) | ADC_SC1_DIFF_MASK | adc_register;
			break;
	};
	
	return;
};

/* sets up pins for the voltage sources on the Shield board */
void init_shield_adc_pins(void){    
 
	set_port_mux(PORTD_PERIPHERAL, CURRENT_SENSE_PIN, CURRENT_SENSE_MUX);
	set_port_mux(PORTD_PERIPHERAL, POTENTIOMETER_PIN, POTENTIOMETER_MUX);
	set_port_mux(PORTD_PERIPHERAL, LIGHT_SENSE_PIN, LIGHT_SENSE_MUX);
	
	return;
}


/* enable the bandgap reference in the KL25Z by turning on it's buffer amplifier. */
void enable_bandgap_reference(void){
	
	PMC_REGSC |= (0x1 << PMC_REGSC_BGBE_SHIFT);
}

/* enable the bus clock in the DAC */
void enable_dac_clock(void){
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK;
	return;
}

/* DAC initialization 
 * 
 * Signal			DAC Output	PORT	Mux
 * -------------	----------	----	---
 * DAC Output:		DAC0_OUT	PTE30	ALT0
 * 
 * The DAC output shares the Servo3 signal on the Shield board.
 * The DAC buffer mode is disabled. The output follows the values in the DAT0L and DAT0H registers.
 * VDDA and VSSA are selected as the reference voltage.
 */ 
void init_dac(void){    
 
	DAC0_C1 = 0;
	DAC0_C2 = 0;
	DAC0_C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	return;
}

/* sets up the DAC output pin */
void init_dac_pin(void){    
	set_port_mux(PORTE_PERIPHERAL, DAC_OUT_PIN, DAC_OUT_MUX);
	return;
}

void set_dac_output(uint16_t data_out){
	DAC0_DAT0L = (uint8_t)( data_out & DAC_DATL_DATA0_MASK);
	DAC0_DAT0H = (uint8_t)((data_out >> 8) & DAC_DATH_DATA1_MASK);
	return;
}

/* sets up the ADC differential input from the Shield board
 * This differential input shares pins with ECHO and ESP_RESET on the Shield board
 */
void init_adc_diff3_pins(void){    
	set_port_mux(PORTE_PERIPHERAL, ADC_DP3_PIN, ADC_DP3_MUX);
	set_port_mux(PORTE_PERIPHERAL, ADC_DM3_PIN, ADC_DM3_MUX);
	return;
}

 /* sets up the ADC single ended input from the Shield board
  * This input shares a pin with the PING header on the Shield board
  */
void init_adc_se4a_pin(void){    
 	set_port_mux(PORTE_PERIPHERAL, ADC_SE4A_PIN, ADC_SE4A_MUX);
 	set_port_mux(PORTE_PERIPHERAL, ADC_SE4A_PIN, ADC_SE4A_MUX);
 	return;
}
  
  /* enable the bus clock in the CMP */
void enable_cmp_clock(void){
  	SIM_SCGC4 |= SIM_SCGC4_CMP_MASK;
  	return;
}

  /* initialize the DAC in the Comparator peripheral
   * Enables the DAC, selects the Vin2 Vref and sets an the initial voltage level to the minimum
   */
void init_cmp_dac(void){
  CMP0_DACCR = (0x1 << CMP_DACCR_VRSEL_SHIFT) | (0x1 << CMP_DACCR_DACEN_SHIFT);
  return;
}

/* intialize the DAC in the Comparator peripheral
 * Enables the DAC, selects the tbd Vref and sets an initial voltage level
 * Parameter:
 * 	level - DAC output level, uint16_t is used to match the other 12 bit DAC.
 * 			Only the lower 6 bits are used.
 */
void set_cmp_dac_output(uint16_t level){
  CMP0_DACCR = (CMP0_DACCR & ~(CMP_DACCR_VOSEL_MASK)) | (((uint8_t)level) & CMP_DACCR_VOSEL_MASK);
  return;
}

/* initialize the comparator in the CMP peripheral
 * Windowing off
 * No interrupts
 * No edge flags
 * Low Speed Mode
 * Input selection is not changed
 * Disables all interrupt sources and clears all interrupt flags
 * Parameters:
 * 	hysteresis - 0 to 3 sets 5mV, 10mV, 20mV or 30mV, like the data sheet
 * 	filter_count - Number of sample periods, like the data sheet
 * 	filter_period - Number of bus clock for the sample period, like the data sheet
 */
void init_cmp(uint8_t hysteresis, uint8_t filter_count, uint8_t filter_period){
	CMP0_CR0 = (hysteresis & CMP_CR0_HYSTCTR_MASK) | ((filter_count << CMP_CR0_FILTER_CNT_SHIFT) & CMP_CR0_FILTER_CNT_MASK);
	CMP0_CR1 = CMP_CR1_OPE_MASK | CMP_CR1_EN_MASK;
	CMP0_FPR = filter_period;
	CMP0_SCR = CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK;	/* interrupt flags are "write 1 to clear". All other bits to 0 */
	CMP0_MUXCR = CMP0_MUXCR | CMP_MUXCR_PSTM_MASK;
	return;
}

/* set the comparator mux input signals
 * Parameters:
 * 	plus_input - channel number of the plus input
 * 	minus_input - channel nuber of the minus input
 * 	
 */
void set_cmp_inputs(int plus_input, int minus_input){
	CMP0_MUXCR = (CMP0_MUXCR & CMP_MUXCR_PSTM_MASK) | (CMP_MUXCR_PSEL(plus_input)) | (CMP_MUXCR_MSEL(minus_input));
	return;
}

/* return the comparator output level, 0 or 1
 */
uint16_t get_cmp_level(void){
	return (CMP0_SCR & CMP_SCR_COUT_MASK);
}

/* sets up the comparator input pin on the Shield board */
void init_shield_cmp_input_pin(void){    
	set_port_mux(PORTE_PERIPHERAL, SHIELD_CMP_INPUT_PIN, SHIELD_CMP_INPUT_MUX);
	return;
}

/* sets up the comparator output pin on the Shield board */
void init_shield_cmp_output_pin(void){    
	set_port_mux(PORTC_PERIPHERAL, SHIELD_CMP_OUTPUT_PIN, SHIELD_CMP_OUTPUT_MUX);
	return;
}

/* sets CMP interrupt conditions
 */
void set_cmp_interrupt_condition(int condition){
	switch (condition){
		case CMP_RISING_EDGE :
			CMP0_SCR = (CMP0_SCR | CMP_SCR_IER_MASK) & ~(CMP_SCR_IEF_MASK);
			break;
		case CMP_FALLING_EDGE :
			CMP0_SCR = (CMP0_SCR | CMP_SCR_IEF_MASK) & ~(CMP_SCR_IER_MASK);
			break;
		case CMP_BOTH_EDGES :
			CMP0_SCR |= (CMP_SCR_IEF_MASK | CMP_SCR_IER_MASK);
			break;
		case CMP_NEITHER_EDGE :
			CMP0_SCR &= ~(CMP_SCR_IEF_MASK | CMP_SCR_IER_MASK);
			break;
	}
	return;
}

/* returns the edge flags with the rising edge in Bit 1 and the falling edge in Bit 0 */
uint16_t get_cmp_edges(void){
	uint16_t temp;
	temp = (uint16_t)(CMP0_SCR >> CMP_SCR_CFF_SHIFT) & 0x1;
	temp = temp | (uint16_t)(((CMP0_SCR >> CMP_SCR_CFR_SHIFT) & 0x1) << 1);
//	return ((uint16_t)CMP0_SCR) & (CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK);
	return temp;
}

/* clear CMP edge flags */
void clear_cmp_edge_flags(int condition){
	switch (condition){
		case CMP_RISING_EDGE :
			CMP0_SCR |=  CMP_SCR_CFR_MASK;
			break;
		case CMP_FALLING_EDGE :
			CMP0_SCR |= CMP_SCR_CFF_MASK;
			break;
		case CMP_BOTH_EDGES :
			CMP0_SCR |= (CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK);
			break;
	}
	return;
}
