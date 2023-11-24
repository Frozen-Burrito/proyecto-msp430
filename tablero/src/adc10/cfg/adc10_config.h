/*
 * adc10_config.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_ADC_CFG_ADC10_CONFIG_H_
#define SRC_ADC_CFG_ADC10_CONFIG_H_

//#include "timer.h"

#define ADC10_CHANNEL_SEQUENCE_ONE_SAMPLE

// A number between 0 and 7 (A0 - A7)
#define ADC10_FIRST_CHANNEL         ((uint8_t) 4u)

#define ADC10_SAMPLE_PERIOD_MS      ((uint8_t) 200u)
#define ADC10_SAMPLE_BUF_LEN        ((uint8_t) 2u)

//#define ADC10_TIMER_SOURCE          (TIMER_A0)
//#define ADC10_TIMER_COUNT           (COUNT_1)

#define ADC10_TIMER_SOURCE          (0u)
#define ADC10_TIMER_COUNT           (1u)

#endif /* SRC_ADC_CFG_ADC10_CONFIG_H_ */
