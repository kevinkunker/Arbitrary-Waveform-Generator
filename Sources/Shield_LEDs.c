/*   Shield_LEDs.c
 * 
 * Shield LED routines
 *
 *  2020-09-27, sac, first one
 *  2020-10-15, sac, added led toggling
 *
 */

#include "derivative.h"
#include "Shield_LEDs.h"
#include "KL25Z_port.h"
#include "KL25Z_gpio.h"

/*	Initialize all four LEDs 
 *	 
 */
void init_shield_leds(void)
{
	set_port_mux(PORTC_PERIPHERAL, LED1_PIN, LED1_MUX);
	set_port_mux(PORTC_PERIPHERAL, LED2_PIN, LED2_MUX);
	set_port_mux(PORTC_PERIPHERAL, LED3_PIN, LED3_MUX);
	set_port_mux(PORTC_PERIPHERAL, LED4_PIN, LED4_MUX);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, LED1_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, LED1_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, LED1_PIN);
	disable_port_pin_pull_resistor(PORTC_PERIPHERAL, LED1_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, LED1_PIN, GPIO_OUT);
	set_gpio_pin_level(PORTC_PERIPHERAL, LED1_PIN, 1);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, LED2_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, LED2_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, LED2_PIN);
	disable_port_pin_pull_resistor(PORTC_PERIPHERAL, LED2_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, LED2_PIN, GPIO_OUT);
	set_gpio_pin_level(PORTC_PERIPHERAL, LED2_PIN, 1);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, LED3_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, LED3_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, LED3_PIN);
	disable_port_pin_pull_resistor(PORTC_PERIPHERAL, LED3_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, LED3_PIN, GPIO_OUT);
	set_gpio_pin_level(PORTC_PERIPHERAL, LED3_PIN, 1);

	disable_port_pin_high_drive(PORTC_PERIPHERAL, LED4_PIN);
	disable_port_pin_filter(PORTC_PERIPHERAL, LED4_PIN);
	select_port_pin_slow_slew(PORTC_PERIPHERAL, LED4_PIN);
	disable_port_pin_pull_resistor(PORTC_PERIPHERAL, LED4_PIN);
	init_gpio_pin(PORTC_PERIPHERAL, LED4_PIN, GPIO_OUT);
	set_gpio_pin_level(PORTC_PERIPHERAL, LED4_PIN, 1);


}

/* lower 4 bits in the argument turn on and off the four Shield LEDs */
void control_all_leds(uint32_t leds){
	if(leds & 0x1) shield_led4_on();
		else shield_led4_off();
	if(leds & 0x2) shield_led3_on();
		else shield_led3_off();
	if(leds & 0x4) shield_led2_on();
		else shield_led2_off();
	if(leds & 0x8) shield_led1_on();
		else shield_led1_off();
	return;
}

void shield_led1_on(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED1_PIN, 0);
	return; 
}
void shield_led2_on(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED2_PIN, 0);
	return; 
}
void shield_led3_on(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED3_PIN, 0);
	return; 
}
void shield_led4_on(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED4_PIN, 0);
	return; 
}
void shield_led1_off(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED1_PIN, 1);
	return; 
}
void shield_led2_off(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED2_PIN, 1);
	return; 
}
void shield_led3_off(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED3_PIN, 1);
	return; 
}
void shield_led4_off(void){
	set_gpio_pin_level(PORTC_PERIPHERAL, LED4_PIN, 1);
	return; 
}
void shield_led1_toggle(void){
	toggle_gpio_pin_level(PORTC_PERIPHERAL, LED1_PIN);
	return; 
}
void shield_led2_toggle(void){
	toggle_gpio_pin_level(PORTC_PERIPHERAL, LED2_PIN);
	return; 
}
void shield_led3_toggle(void){
	toggle_gpio_pin_level(PORTC_PERIPHERAL, LED3_PIN);
	return; 
}
void shield_led4_toggle(void){
	toggle_gpio_pin_level(PORTC_PERIPHERAL, LED4_PIN);
	return; 
}
