/*
 * KL25Z_pit.h
 *
 *  Created on: Sep 4, 2020
 *      Author: steve
 */

#ifndef KL25Z_PIT_H_
#define KL25Z_PIT_H_

enum PIT_TIMERS {
				PIT_TIMER_0,
				PIT_TIMER_1
				};

enum PIT_INTERRUPTS {
				PIT_INT_OFF,
				PIT_INT_ON
				};

void enable_pit_clock(void);
void init_PIT(int timer_select, uint32_t modulus, int interrupts);
void set_PIT_modulus(int timer_select, int interrupts);
void start_PIT(int timer_select);
void stop_PIT(int timer_select);
uint32_t get_PIT_count(int timer_select);
uint32_t get_PIT_int_flag(int timer_select);
void clear_PIT_int_flag(int timer_select);

#endif /* KL25Z_PIT_H_ */
