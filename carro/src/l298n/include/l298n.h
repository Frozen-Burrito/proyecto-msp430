/*
 * l298n.h
 *
 *  Created on: Nov 30, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_L298N_INCLUDE_L298N_H_
#define SRC_L298N_INCLUDE_L298N_H_

#include <stdint.h>
#include "timer.h"

#define MOTOR_CHANNEL_A ((uint8_t) 0x01u)
#define MOTOR_CHANNEL_B ((uint8_t) 0x02u)

#define MOTOR_DIRECTION_FORWARD (1u)
#define MOTOR_DIRECTION_REVERSE (2u)

void l298n_init(uint8_t motor_channels);

void l298n_motor_set_speed(uint8_t motor_channels, uint8_t speed, uint8_t direction);

#endif /* SRC_L298N_INCLUDE_L298N_H_ */
