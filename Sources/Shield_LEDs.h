
/*  Shield_LEDs.h
 * 
 *  Shield pushbutton routines
 *  
 *  2020-09-25, sac, released
 *  2020-10-04, sac, added control_all_leds()
 *  2020-10-15, sac, added led toggling prototypes
 */

#ifndef Shield_LEDs_H_
#define Shield_LEDs_H_

#define LED1_PIN				12
#define LED1_MUX				ALT1
#define LED2_PIN				13
#define LED2_MUX				ALT1
#define LED3_PIN				16
#define LED3_MUX				ALT1
#define LED4_PIN				17
#define LED4_MUX				ALT1

/* Function prototypes */
void init_shield_leds(void);
void control_all_leds(uint32_t leds);
void shield_led1_on(void);
void shield_led2_on(void);
void shield_led3_on(void);
void shield_led4_on(void);
void shield_led1_off(void);
void shield_led2_off(void);
void shield_led3_off(void);
void shield_led4_off(void);
void shield_led1_toggle(void);
void shield_led2_toggle(void);
void shield_led3_toggle(void);
void shield_led4_toggle(void);

#endif // Shield_LEDs_H_

