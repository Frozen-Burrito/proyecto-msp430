/*
 * velocity_sensor.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_VELOCITY_SENSOR_INCLUDE_VELOCITY_SENSOR_H_
#define SRC_VELOCITY_SENSOR_INCLUDE_VELOCITY_SENSOR_H_

#include <stdint.h>

// El tiempo (us) que tarda en girar 1/20 de revolucion la rueda.
uint32_t rev_fraction_period_us;

void velocity_sensor_init(void);

#endif /* SRC_VELOCITY_SENSOR_INCLUDE_VELOCITY_SENSOR_H_ */
