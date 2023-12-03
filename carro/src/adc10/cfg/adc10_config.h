/*
 * adc10_config.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_ADC_CFG_ADC10_CONFIG_H_
#define SRC_ADC_CFG_ADC10_CONFIG_H_

#include "timer.h"

//#define ADC10_CHANNEL_SEQUENCE_ONE_SAMPLE

// A number between 0 and 7 (A0 - A7)
#define ADC10_FIRST_CHANNEL         ((uint8_t) 0u)

#define ADC10_SAMPLE_PERIOD_MS      ((uint16_t) 5000u)
#define ADC10_SAMPLE_BUF_LEN        ((uint8_t) 1u)

#define ADC10_USE_DEDICATED_TIMER

#ifdef ADC10_USE_DEDICATED_TIMER
#define ADC10_TIMER_SOURCE          (TIMER_A0)
#define ADC10_TIMER_COUNT           (COUNT_1)
#endif /* ADC10_USE_DEDICATED_TIMER */

#endif /* SRC_ADC_CFG_ADC10_CONFIG_H_ */
