/*
 * controls_config.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_CONTROLS_CFG_CONTROLS_CONFIG_H_
#define SRC_CONTROLS_CFG_CONTROLS_CONFIG_H_

#define STEERING_WHEEL_PIN  (0x0008u)

#define PEDAL_PIN           (0x0010u)

#define SUPPORT_REVERSE

#ifdef SUPPORT_REVERSE
#define REVERSE_TOGGLE_PIN  (0x0001u)
#endif

#endif /* SRC_CONTROLS_CFG_CONTROLS_CONFIG_H_ */
