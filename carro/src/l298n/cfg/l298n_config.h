/*
 * l298n_config.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_L298N_CFG_L298N_CONFIG_H_
#define SRC_L298N_CFG_L298N_CONFIG_H_
#include <msp430.h>
#include "timer.h"

#define MOTOR_PWM_PERIOD_US     (1000u - 1u)
#define MOTOR_PWM_TIMER_SOURCE  (TIMER_A1)

#define MOTOR_A_PWM_PIN         ((uint8_t) 0x02u)
#define MOTOR_A_PWM_PORT        ((uint8_t) 2u)

#define MOTOR_B_PWM_PIN         ((uint8_t) 0x20u)
#define MOTOR_B_PWM_PORT        ((uint8_t) 2u)

//#define MOTOR_FORWARD_ONLY      (1u)

#ifndef MOTOR_FORWARD_ONLY
#define MOTOR_IN1_PIN           (BIT3)
#define MOTOR_IN2_PIN           (BIT7)
#endif

#endif /* SRC_L298N_CFG_L298N_CONFIG_H_ */
