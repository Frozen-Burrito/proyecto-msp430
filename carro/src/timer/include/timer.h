/*
 * timer.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_TIMER_INCLUDE_TIMER_H_
#define SRC_TIMER_INCLUDE_TIMER_H_

#include <stdint.h>

#define TIMER_A0    (0u)
#define TIMER_A1    (1u)

#define COUNT_1     (0u)
#define COUNT_2     (1u)
#define COUNT_3     (2u)

typedef void (*timer_callback_t)(void);

void timer_input_capture(uint8_t timer_id, uint8_t count_id, uint8_t pin);

void timer_pwm_init(uint8_t timer_id, uint16_t period_us, uint8_t pin_mask);

void timer_start(uint8_t timer_id, uint8_t count_id, uint16_t period_ms, timer_callback_t cb);

void timer_execute_pending_callbacks(void);

#endif /* SRC_TIMER_INCLUDE_TIMER_H_ */
