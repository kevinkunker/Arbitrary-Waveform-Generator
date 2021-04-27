/*   KL25Z_port.c
 * 
 * PORT initialization and control
 * 
 * Note: There are no pulldown resistors on the KL25Z.
 *       So, there are no functions for control of the PCR PS bit 
 */

#include "derivative.h"
#include "KL25Z_port.h"

/* Enable a PORT module bus clock
 */
void enable_port_clock(int port_id)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			break;
		case PORTB_PERIPHERAL :
			SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
			break;
		case PORTC_PERIPHERAL :
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    		break;
		case PORTD_PERIPHERAL :
			SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
			break;
		case PORTE_PERIPHERAL :
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
    		break;
	}
	return;
}


/* set the MUX field in the PCR (peripheral control register) */
void set_port_mux(int port_id, int pin, int mux_field)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin) & ~PORT_PCR_MUX_MASK) | (PORT_PCR_MUX(mux_field));
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin) & ~PORT_PCR_MUX_MASK) | (PORT_PCR_MUX(mux_field));
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin) & ~PORT_PCR_MUX_MASK) | (PORT_PCR_MUX(mux_field));
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin) & ~PORT_PCR_MUX_MASK) | (PORT_PCR_MUX(mux_field));
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin) & ~PORT_PCR_MUX_MASK) | (PORT_PCR_MUX(mux_field));
    		break;
	}
	return;
}

/* enable high drive strength of a Port pin */
void enable_port_pin_high_drive(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) | (PORT_PCR_DSE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) | (PORT_PCR_DSE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) | (PORT_PCR_DSE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) | (PORT_PCR_DSE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) | (PORT_PCR_DSE_MASK);
    		break;
	}
	return;
}

/* disable high drive strength of a Port pin */
void disable_port_pin_high_drive(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) & ~(PORT_PCR_DSE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) & ~(PORT_PCR_DSE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) & ~(PORT_PCR_DSE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) & ~(PORT_PCR_DSE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) & ~(PORT_PCR_DSE_MASK);
    		break;
	}
	return;
}

/* enable passive filter of a Port pin */
void enable_port_pin_filter(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) | (PORT_PCR_PFE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) | (PORT_PCR_PFE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) | (PORT_PCR_PFE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) | (PORT_PCR_PFE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) | (PORT_PCR_PFE_MASK);
    		break;
	}
	return;
}

/* disable high drive strength of a Port pin */
void disable_port_pin_filter(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) & ~(PORT_PCR_PFE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) & ~(PORT_PCR_PFE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) & ~(PORT_PCR_PFE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) & ~(PORT_PCR_PFE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) & ~(PORT_PCR_PFE_MASK);
    		break;
	}
	return;
}

/* select slow slew rate for a port pin */
void select_port_pin_slow_slew(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) | (PORT_PCR_SRE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) | (PORT_PCR_SRE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) | (PORT_PCR_SRE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) | (PORT_PCR_SRE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) | (PORT_PCR_SRE_MASK);
    		break;
	}
	return;
}

/* select fast slew rate for a port pin */
void select_port_pin_fast_slew(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) & ~(PORT_PCR_SRE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) & ~(PORT_PCR_SRE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) & ~(PORT_PCR_SRE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) & ~(PORT_PCR_SRE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) & ~(PORT_PCR_SRE_MASK);
    		break;
	}
	return;
}

/* enable the pull resistor on a port pin */
void enable_port_pin_pull_resistor(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) | (PORT_PCR_PE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) | (PORT_PCR_PE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) | (PORT_PCR_PE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) | (PORT_PCR_PE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) | (PORT_PCR_PE_MASK);
    		break;
	}
	return;
}

/* disable the pull resistor on a port pin */
void disable_port_pin_pull_resistor(int port_id, int pin)
{
	switch (port_id){
		case PORTA_PERIPHERAL :  
			PORTA_PCR(pin) = (PORTA_PCR(pin)) & ~(PORT_PCR_PE_MASK);
    		break;
		case PORTB_PERIPHERAL :
			PORTB_PCR(pin) = (PORTB_PCR(pin)) & ~(PORT_PCR_PE_MASK);
			break;
		case PORTC_PERIPHERAL :
			PORTC_PCR(pin) = (PORTC_PCR(pin)) & ~(PORT_PCR_PE_MASK);
    		break;
		case PORTD_PERIPHERAL :
			PORTD_PCR(pin) = (PORTD_PCR(pin)) & ~(PORT_PCR_PE_MASK);
			break;
		case PORTE_PERIPHERAL :
			PORTE_PCR(pin) = (PORTE_PCR(pin)) & ~(PORT_PCR_PE_MASK);
    		break;
	}
	return;
}
