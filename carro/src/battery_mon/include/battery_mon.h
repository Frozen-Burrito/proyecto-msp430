/*
 * battery_mon.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_BATTERY_MON_INCLUDE_BATTERY_MON_H_
#define SRC_BATTERY_MON_INCLUDE_BATTERY_MON_H_

#include <stdint.h>
#include "adc10.h"

void battery_mon_init(void);

void battery_mon_sample(void);

uint16_t battery_mon_get_voltage(void);

#endif /* SRC_BATTERY_MON_INCLUDE_BATTERY_MON_H_ */
