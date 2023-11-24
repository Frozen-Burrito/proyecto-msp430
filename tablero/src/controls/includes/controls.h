/*
 * controls.h
 *
 *  Created on: Nov 18, 2023
 *      Author: f3rm3
 */

#ifndef SRC_CONTROLS_INCLUDES_CONTROLS_H_
#define SRC_CONTROLS_INCLUDES_CONTROLS_H_

#include "../../adc10/include/adc10.h"

void controls_init(void);

uint8_t controls_get_state(uint8_t * out_speed, uint16_t * out_direction);

#endif /* SRC_CONTROLS_INCLUDES_CONTROLS_H_ */
