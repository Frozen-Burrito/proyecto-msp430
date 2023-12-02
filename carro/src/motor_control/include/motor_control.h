/*
 * motor_control.h
 *
 *  Created on: Dec 1, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_
#include <stdint.h>
#include "l298n.h"
#include "l298n_config.h"

void motor_control_init(void);

void motor_control(uint16_t steering, uint8_t speed);

#endif /* SRC_MOTOR_CONTROL_INCLUDE_MOTOR_CONTROL_H_ */
