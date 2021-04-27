/*
 * KL25Z_pit.c
 *
 *  Created on: Sep 12, 2020
 *      Author: sac
 *      
 */

#include <stdint.h>
#include "MKL25Z4.h"
#include "KL25Z_pit.h"


void enable_pit_clock(void){
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	return;
}

/* Initialize a PIT with a modulus
 *
 * The timer is set to stop in debug mode.
 * Timer chaining is not supported.
 * The timer is disabled before any changes are made.
 * 
 */
void init_PIT(int timer_select, uint32_t modulus, int interrupts){

	PIT_MCR = PIT_MCR & ~(PIT_MCR_MDIS_MASK);  /* enable the module */ 
	PIT_MCR = PIT_MCR | PIT_MCR_FRZ_MASK;      /* set to stop in debug */
	
	if(timer_select == PIT_TIMER_0){
		PIT_TCTRL0 = PIT_TCTRL0 & ~(PIT_TCTRL_TEN_MASK);  /* disable the timer */
		PIT_TCTRL0 = PIT_TCTRL0 & ~(PIT_TCTRL_CHN_MASK);	/* turn off chaining */
		PIT_LDVAL0 = PIT_LDVAL_TSV(modulus);			/* load the new modulus */
		if(interrupts == PIT_INT_OFF) PIT_TCTRL0 = PIT_TCTRL0 & ~(PIT_TCTRL_TIE_MASK);
		else  PIT_TCTRL0 = PIT_TCTRL0 | (PIT_TCTRL_TIE_MASK);
	};
	if(timer_select == PIT_TIMER_1){
		PIT_TCTRL1 = PIT_TCTRL1 & ~(PIT_TCTRL_TEN_MASK);
		PIT_TCTRL1 = PIT_TCTRL1 & ~(PIT_TCTRL_CHN_MASK);
		PIT_LDVAL1 = PIT_LDVAL_TSV(modulus);
		PIT_CVAL1 = PIT_CVAL1 & ~(PIT_TCTRL_CHN_MASK);
		if(interrupts == PIT_INT_OFF) PIT_TCTRL1 = PIT_TCTRL1 & ~(PIT_TCTRL_TIE_MASK);
		else  PIT_TCTRL1 = PIT_TCTRL1 | (PIT_TCTRL_TIE_MASK);
	};
}

/* loads a new PIT modulus
 *
 * Does not stop the timer 
 */
void set_PIT_modulus(int timer_select, int modulus){
	
	if(timer_select == PIT_TIMER_0)
		PIT_LDVAL0 = PIT_LDVAL_TSV(modulus);
	if(timer_select == PIT_TIMER_1)
		PIT_LDVAL1 = PIT_LDVAL_TSV(modulus);
}

void start_PIT(int timer_select){
	
	if(timer_select == PIT_TIMER_0)
		PIT_TCTRL0 = PIT_TCTRL0 | (PIT_TCTRL_TEN_MASK);
	if(timer_select == PIT_TIMER_1)
		PIT_TCTRL1 = PIT_TCTRL1 | (PIT_TCTRL_TEN_MASK);
}

void stop_PIT(int timer_select){
	
	if(timer_select == PIT_TIMER_0)
		PIT_TCTRL0 = PIT_TCTRL0 & ~(PIT_TCTRL_TEN_MASK);
	if(timer_select == PIT_TIMER_1)
		PIT_TCTRL1 = PIT_TCTRL1 & ~(PIT_TCTRL_TEN_MASK);
}

uint32_t get_PIT_count(int timer_select){
	
	uint32_t count;
	if(timer_select == PIT_TIMER_0)
		count = PIT_CVAL0;
	if(timer_select == PIT_TIMER_1)
		count = PIT_CVAL1;
	return count;
}

uint32_t get_PIT_int_flag(int timer_select){
	
	uint32_t flag;
	if(timer_select == PIT_TIMER_0)
		flag = (PIT_TFLG0 & PIT_TFLG_TIF_MASK) >> PIT_TFLG_TIF_SHIFT;
	if(timer_select == PIT_TIMER_1)
		flag = (PIT_TFLG1 & PIT_TFLG_TIF_MASK) >> PIT_TFLG_TIF_SHIFT;
	return flag;
}


void clear_PIT_int_flag(int timer_select){
	
	if(timer_select == PIT_TIMER_0)
		PIT_TFLG0 = PIT_TFLG0 | PIT_TFLG_TIF_MASK; /* write a 1 to clear */
	if(timer_select == PIT_TIMER_1)
		PIT_TFLG1 = PIT_TFLG1 | PIT_TFLG_TIF_MASK;	
	
	return;
}

