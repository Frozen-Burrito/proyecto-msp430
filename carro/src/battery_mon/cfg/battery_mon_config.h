/*
 * battery_mon_config.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_BATTERY_MON_CFG_BATTERY_MON_CONFIG_H_
#define SRC_BATTERY_MON_CFG_BATTERY_MON_CONFIG_H_

#define BATTERY_ADC_MAX_MV  ((uint16_t) 3200u)
#define BATTERY_ADC_PIN     ((uint16_t) 0x0000u)

#define BATTERY_MIN_MV      ((uint16_t) 3600u)
#define BATTERY_MAX_MV      ((uint16_t) 6400u)

#define BATTERY_MON_SAMPLE_PERIOD_MS    ((uint16_t) 5000u)

#endif /* SRC_BATTERY_MON_CFG_BATTERY_MON_CONFIG_H_ */
