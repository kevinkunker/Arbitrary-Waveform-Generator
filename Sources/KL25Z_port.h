
/*  KL25Z_port.h
 * 
 *  PORT initialization and control for the KL25Z
 * 
 * 	Note: There is no control of the PCR PS bit. There are not pulldowns
 * 	on the KL25Z
 */

#ifndef KL25Z_port_H_
#define KL25Z_port_H_

/* Port peripheral selections */
enum port_selects {
	PORTA_PERIPHERAL,
	PORTB_PERIPHERAL,
	PORTC_PERIPHERAL,
	PORTD_PERIPHERAL,
	PORTE_PERIPHERAL
};

/* possible values in the MUX field of the port PCR */
enum port_mux_settings {
					ALT0 = 0,	/* pin disabled */
					ALT1 = 1,	/* GPIO */
					ALT2 = 2,	/* see signal mux table */
					ALT3 = 3,	/* see signal mux table */
					ALT4 = 4,	/* see signal mux table */
					ALT5 = 5,	/* see signal mux table */
					ALT6 = 6,	/* see signal mux table */
					ALT7 = 7	/* see signal mux table */
};

/* Function prototypes */
void enable_port_clock(int port_id);
void set_port_mux(int port_id, int pin, int mux_field);
void enable_port_pin_high_drive(int port_id, int pin);
void disable_port_pin_high_drive(int port_id, int pin);
void enable_port_pin_filter(int port_id, int pin);
void disable_port_pin_filter(int port_id, int pin);
void select_port_pin_slow_slew(int port_id, int pin);
void select_port_pin_fast_slew(int port_id, int pin);
void enable_port_pin_pull_resistor(int port_id, int pin);
void disable_port_pin_pull_resistor(int port_id, int pin);
/* Note: no control of the Pull Select (PS) bit. There are no pulldowns on the KL25Z */



#endif // KL25Z_port_H_

