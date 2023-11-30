/*
 * adc10.h
 *
 *  Created on: Nov 18, 2023
 *      Author: f3rm3
 */

#ifndef SRC_ADC_INCLUDES_ADC10_H_
#define SRC_ADC_INCLUDES_ADC10_H_

#include <stdint.h>

#include "adc10_config.h"
#include "timer.h"

#define ADC10_INITIALIZED       (0x0001)
#define ADC10_SAMPLES_READY_FG  (0x0002)

volatile uint8_t adc10_flags;

volatile uint16_t adc10_samples[ADC10_SAMPLE_BUF_LEN] = {};

void adc10_init(uint8_t adc_inputs);

void adc10_start_conversion(void);

#endif /* SRC_ADC_INCLUDES_ADC10_H_ */
