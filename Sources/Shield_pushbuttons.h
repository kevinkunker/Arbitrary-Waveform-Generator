
/*  Shield_buttons.h
 * 
 *  Shield pushbutton routines
 *  
 *  2020-09-27, sac, released
 *  2021-03-08, sac, derived from Shield_buttons.h,
 *  				removed stuff for the button monitor/debouncer
 * 
 */

#ifndef Shield_pushbuttons_H_
#define Shield_pushbuttons_H_

#define PB1_PIN				4
#define PB1_MUX				ALT1
#define PB2_PIN				5
#define PB2_MUX				ALT1
#define PB3_PIN				6
#define PB3_MUX				ALT1
#define PB4_PIN				7
#define PB4_MUX				ALT1

enum SHIELD_BUTTON_ID {
	SHIELD_PB1 = 0,
	SHIELD_PB2 = 1,
	SHIELD_PB3 = 2,
	SHIELD_PB4 = 3
};

/* Function prototypes */
void init_shield_buttons(void);
int get_shield_pb(int button);
int get_shield_pb1(void);
int get_shield_pb2(void);
int get_shield_pb3(void);
int get_shield_pb4(void);

#endif // Shield_pushbuttons_H_

