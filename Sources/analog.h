/*
 * analog.h
 *
 *  Created on: Oct 6, 2020
 *      Author: sac
 *	2020-10-14, sac, added CMP stuff
 *	2021-03-26, sac, added convert_adc_with_irq() prototype
 *	                 added setup_adc() prototype
 *	2021-04-03, sac, added comment about ADC channel & mux reference
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#define CURRENT_SENSE_PIN	1
#define CURRENT_SENSE_MUX	ALT0
#define POTENTIOMETER_PIN	5
#define POTENTIOMETER_MUX	ALT0
#define LIGHT_SENSE_PIN		6
#define LIGHT_SENSE_MUX		ALT0
#define DAC_OUT_PIN			30
#define DAC_OUT_MUX			ALT0
#define ADC_DP3_PIN			22
#define ADC_DP3_MUX			ALT0
#define ADC_DM3_PIN			23
#define ADC_DM3_MUX			ALT0
#define ADC_SE4A_PIN		21
#define ADC_SE4A_MUX		ALT0
#define SHIELD_CMP_INPUT_PIN		29
#define SHIELD_CMP_INPUT_MUX		ALT0
#define SHIELD_CMP_OUTPUT_PIN		0
#define SHIELD_CMP_OUTPUT_MUX		ALT5

enum shield_adc_mode {		// values match the MODE bit field in the ADCx_CFG1 register
	ADC_8BIT_RESOLUTION = 0,
	ADC_10BIT_RESOLUTION = 2,
	ADC_12BIT_RESOLUTION = 1,
	ADC_16BIT_RESOLUTION = 3
};


/******************************************************
 * ADC Channel and MUX Assignments
 * See KL25Z Reference Manual Section 3.7.1.3 for
 *   channel/mux combinations
 ******************************************************/
enum named_adc_channels {
	SHIELD_CURRENT_SENSE = 5,
	SHIELD_TRIMPOT = 6,
	SHIELD_LIGHT_SENSOR = 7,
	KL25Z_TEMPERATURE = 26,
	KL25Z_BG_REFERENCE = 27,  /* Band Gap must be turned on! */
	KL25Z_ADC_DIFF3 = 3,
	KL25Z_ADC_SE4A = 4
};

enum adc_mux {
	ADC_INPUT_SINGLE,
	ADC_INPUT_SINGLE_A,
	ADC_INPUT_SINGLE_B,
	ADC_INPUT_DIFFERENTIAL
};
/******************************************************/


enum dac_codes {
	DAC_12BIT_0_0_VOLTS = 0,
	DAC_12BIT_0_5_VOLTS = 621,
	DAC_12BIT_1_0_VOLTS = 1241,
	DAC_12BIT_1_5_VOLTS = 1862,
	DAC_12BIT_2_0_VOLTS = 2482,
	DAC_12BIT_2_5_VOLTS = 3103,
	DAC_12BIT_3_0_VOLTS = 3724,
};

enum named_cmp_input_channels {
	CMP_INPUT_SHIELD_PTE29 = 5,
	CMP_INPUT_DAC_12BIT = 4,
	CMP_INPUT_BG_REFERENCE = 6,
	CMP_INPUT_DAC_6BIT = 7
};

enum cmp_edge_conditions {
	CMP_RISING_EDGE, 
	CMP_FALLING_EDGE,
	CMP_BOTH_EDGES,
	CMP_NEITHER_EDGE
};

enum cmp_hysteresis_levels {
	CMP_HYSTERESIS_5mV = 0, 
	CMP_HYSTERESIS_10mV = 1,
	CMP_HYSTERESIS_20mV = 2,
	CMP_HYSTERESIS_30mV = 3
};

void enable_adc_clock(void);
void init_adc(uint32_t resolution);    
void init_shield_adc_pins(void);
uint32_t convert_adc(uint32_t channel, uint32_t input_type);    
void convert_adc_with_irq(uint32_t channel, uint32_t input_type);    
void setup_adc(uint32_t channel, uint32_t input_type,
		       uint32_t interrupt, uint32_t trigger,
		       uint32_t dma);
void enable_bandgap_reference(void);

void enable_dac_clock(void);
void init_dac_pin(void);
void init_dac(void);    
void set_dac_output(uint16_t data_out);
void init_adc_diff3_pins(void);
void init_adc_se4a_pin(void);

void enable_cmp_clock(void);
void init_cmp_dac(void);
void set_cmp_dac_output(uint16_t level);
void init_cmp(uint8_t hysteresis, uint8_t filter_count, uint8_t filter_period);
void set_cmp_inputs(int plus_input, int minus_input);
uint16_t get_cmp_level(void);
void init_shield_cmp_input_pin(void);    
void init_shield_cmp_output_pin(void);    
void set_cmp_interrupt_condition(int condition);
uint16_t get_cmp_edges(void);
void clear_cmp_edge_flags(int condition);

#endif /* ANALOG_H_ */


