/*
 * timer_config.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_TIMER_CFG_TIMER_CONFIG_H_
#define SRC_TIMER_CFG_TIMER_CONFIG_H_

#define MS_PER_TICK     ((uint16_t) 1000u)

#define MS_TO_TICKS(milliseconds) (((uint16_t) milliseconds) / MS_PER_TICK)

#endif /* SRC_TIMER_CFG_TIMER_CONFIG_H_ */
