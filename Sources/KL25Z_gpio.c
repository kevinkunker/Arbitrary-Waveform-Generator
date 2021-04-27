/*   KL25Z__gpio.c
 * 
 * GPIO modules initialization and control
 *
 *  2020-09-13, sac, added get_gpio_output_reg_bit( )
 *  2020-09-18, sac, fixed toggle _gpio_pin_level(). changed "|=" to "="
 */

#include "derivative.h"
#include "KL25Z_gpio.h"


/*	Initialize GPIO module "pins", module pins not device pins
 *	 
 *	gpio_id - which peripheral A, B, C, D or E
 *	pin - pin number of the module from 0 to 31
 *	direction - specifies the direction for a pin 
 *				non-zero - output
 *				0 - input
 */
void init_gpio_pin(int gpio_id, int pin, int direction)
{
	switch (gpio_id){
		case GPIOA_PERIPHERAL :  
			if(direction) GPIOA_PDDR |= 1 << pin;
			else GPIOA_PDDR &= ~(1 << pin);
    		break;
		case GPIOB_PERIPHERAL :
			if(direction) GPIOB_PDDR |= 1 << pin;
			else GPIOB_PDDR &= ~(1 << pin);
			break;
		case GPIOC_PERIPHERAL :
			if(direction) GPIOC_PDDR |= 1 << pin;
			else GPIOC_PDDR &= ~(1 << pin);
    		break;
		case GPIOD_PERIPHERAL :
			if(direction) GPIOD_PDDR |= 1 << pin;
			else GPIOD_PDDR &= ~(1 << pin);
			break;
		case GPIOE_PERIPHERAL :
			if(direction) GPIOE_PDDR |= 1 << pin;
			else GPIOE_PDDR &= ~(1 << pin);
    		break;
	}
	return;
}

/* Get the GPIO pin input level
 * 
 *  gpio_id - which GPIO peripheral, A, B, C, D or E
 *	pin - pin number of the module from 0 to 31
 *	Returns 0 if the pin is low and 1 if the pin is high
 * 
 */
int get_gpio_pin_level(int gpio_id, int pin)
{
	int level = 0;
	switch (gpio_id){
		case GPIOA_PERIPHERAL :  
			if (GPIOA_PDIR & (1 << pin)) level = 1;
			break;
		case GPIOB_PERIPHERAL :
			if (GPIOB_PDIR & (1 << pin)) level = 1;
			break;
		case GPIOC_PERIPHERAL :
			if (GPIOC_PDIR & (1 << pin)) level = 1;
    		break;
		case GPIOD_PERIPHERAL :
			if (GPIOD_PDIR & (1 << pin)) level = 1;
			break;
		case GPIOE_PERIPHERAL :
			if (GPIOE_PDIR & (1 << pin)) level = 1;
			break;
	}
    return level;
}


/* Set the GPIO pin output level
 * 
 *  gpio_id - which GPIO peripheral, A, B, C, D or E
 *	pin - pin number of the module from 0 to 31
 *	level -  0 sets the pin low, non-zero sets the pin high
 */
void set_gpio_pin_level(int gpio_id, int pin, int level)
{
	switch (gpio_id){
		case GPIOA_PERIPHERAL :  
			if (level) GPIOA_PDOR |= 1 << pin;
			else GPIOA_PDOR &= ~(1 << pin);
			break;
		case GPIOB_PERIPHERAL :
			if (level) GPIOB_PDOR |= 1 << pin;
			else GPIOB_PDOR &= ~(1 << pin);
			break;
		case GPIOC_PERIPHERAL :
			if (level) GPIOC_PDOR |= 1 << pin;
			else GPIOC_PDOR &= ~(1 << pin);
    		break;
		case GPIOD_PERIPHERAL :
			if (level) GPIOD_PDOR |= 1 << pin;
			else GPIOD_PDOR &= ~(1 << pin);
			break;
		case GPIOE_PERIPHERAL :
			if (level) GPIOE_PDOR |= 1 << pin;
			else GPIOE_PDOR &= ~(1 << pin);
    		break;
	}
	return;
}

/* Toggle the GPIO pin output level
 * 
 *  gpio_id - which GPIO peripheral, A, B, C, D or E
 *	pin - pin number of the module from 0 to 31
  */
void toggle_gpio_pin_level(int gpio_id, int pin)
{
	switch (gpio_id){
		case GPIOA_PERIPHERAL :  
			GPIOA_PTOR = 1 << pin;
			break;
		case GPIOB_PERIPHERAL :
			GPIOB_PTOR = 1 << pin;
			break;
		case GPIOC_PERIPHERAL :
			GPIOC_PTOR = 1 << pin;
    		break;
		case GPIOD_PERIPHERAL :
			GPIOD_PTOR = 1 << pin;
			break;
		case GPIOE_PERIPHERAL :
			GPIOE_PTOR = 1 << pin;
    		break;
	}
	return;
}

/* get the value of a GPIO output data register bit
 * 
 *  gpio_id - which GPIO peripheral, A, B, C, D or E
 *	bit - bit number of the register from 0 to 31
 */
int get_gpio_output_reg_bit(int gpio_id, int bit)
{
	int bit_state;
	
	switch (gpio_id){
		case GPIOA_PERIPHERAL :  
			bit_state = (GPIOA_PDOR & (1 << bit)) >> bit;
			break;
		case GPIOB_PERIPHERAL :
			bit_state = (GPIOB_PDOR & (1 << bit)) >> bit;
			break;
		case GPIOC_PERIPHERAL :
			bit_state = (GPIOC_PDOR & (1 << bit)) >> bit;
    		break;
		case GPIOD_PERIPHERAL :
			bit_state = (GPIOD_PDOR & (1 << bit)) >> bit;
			break;
		case GPIOE_PERIPHERAL :
			bit_state = (GPIOE_PDOR & (1 << bit)) >> bit;
    		break;
	}
	return bit_state;
}
