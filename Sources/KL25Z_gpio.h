
/*  KL25Z_gpio.h
 * 
 *  GPIO initialization and control for the KL25Z
 *  
 *  2020-09-13, sac, added get_gpio_output_reg_bit( )
 * 
 */


#ifndef KL25Z_gpio_H_
#define KL25Z_gpio_H_

#define GPIO_IN 0
#define GPIO_OUT 1

/* GPIO peripheral selections */
enum gpio_selects {
	GPIOA_PERIPHERAL,
	GPIOB_PERIPHERAL,
	GPIOC_PERIPHERAL,
	GPIOD_PERIPHERAL,
	GPIOE_PERIPHERAL
};

/* Function prototypes */
void init_gpio_pin(int gpio_id, int pin, int direction);
int get_gpio_pin_level(int gpio_id, int pin);
void set_gpio_pin_level(int gpio_id, int pin, int level);
void toggle_gpio_pin_level(int gpio_id, int pin);
int get_gpio_output_reg_bit(int gpio_id, int bit);

#endif // KL25Z_gpio_H_

