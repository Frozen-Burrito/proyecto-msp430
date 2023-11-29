/*
 * radio.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_RADIO_INCLUDE_RADIO_H_
#define SRC_RADIO_INCLUDE_RADIO_H_

#include <stdint.h>
#include "msprf24.h"

void radio_init(uint8_t channel);

uint8_t radio_receive(uint8_t * const out_data, uint8_t data_len);

void radio_transmit(const uint8_t * const data, uint8_t data_len);

#endif /* SRC_RADIO_INCLUDE_RADIO_H_ */
