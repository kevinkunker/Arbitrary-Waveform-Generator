/*   Shield_buttons.c
 * 
 * Shield pushbuttons routines
 *
 *  2020-09-25, sac, released
 *  2020-09-29, sac, added shield button monitor
 *  2021-03-07, sac, derived from Shield_buttons.c, removed the button monitor/debouncer
 */

#include "derivative.h"
#include "Shield_pushbuttons.h"
#include "KL25Z_port.h"
#include "KL25Z_gpio.h"


/*	Initialize all four buttons 
 *	 
 */
void init_shield_buttons(void)
{
	set_port_mux(PORTC_PERIPHERAL, PB1_PIN, PB1_MUX);
	set_port_mux(PORTC_PERIPHERAL, PB2_PIN, PB2_MUX);
	set_port_mux(PORTC_PERIPHERAL, PB3_PIN, PB3_MUX);
	set_port_mux(PORTC_PERIPHERAL, PB4_PIN, PB4_MUX);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, PB1_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, PB1_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, PB1_PIN);
	enable_port_pin_pull_resistor(PORTC_PERIPHERAL, PB1_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, PB1_PIN, GPIO_IN);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, PB2_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, PB2_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, PB2_PIN);
	enable_port_pin_pull_resistor(PORTC_PERIPHERAL, PB2_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, PB2_PIN, GPIO_IN);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, PB3_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, PB3_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, PB3_PIN);
	enable_port_pin_pull_resistor(PORTC_PERIPHERAL, PB3_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, PB3_PIN, GPIO_IN);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, PB4_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, PB4_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, PB4_PIN);
	enable_port_pin_pull_resistor(PORTC_PERIPHERAL, PB4_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, PB4_PIN, GPIO_IN);
}

int get_shield_pb(int button){
	int level;
	switch (button){
		case SHIELD_PB1:
			level = get_gpio_pin_level(PORTC_PERIPHERAL, PB1_PIN);
			break;
		case SHIELD_PB2:
			level = get_gpio_pin_level(PORTC_PERIPHERAL, PB2_PIN);
			break;
		case SHIELD_PB3:
			level = get_gpio_pin_level(PORTC_PERIPHERAL, PB3_PIN);
			break;
		case SHIELD_PB4:
			level = get_gpio_pin_level(PORTC_PERIPHERAL, PB4_PIN);
			break;
	}
	return level;
}

int get_shield_pb1(void){
	
	return get_gpio_pin_level(PORTC_PERIPHERAL, PB1_PIN);
}

int get_shield_pb2(void){
	
	return get_gpio_pin_level(PORTC_PERIPHERAL, PB2_PIN);
}

int get_shield_pb3(void){
	
	return get_gpio_pin_level(PORTC_PERIPHERAL, PB3_PIN);
}

int get_shield_pb4(void){
	
	return get_gpio_pin_level(PORTC_PERIPHERAL, PB4_PIN);
}

