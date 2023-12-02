/*
 * radio.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_RADIO_INCLUDE_RADIO_H_
#define SRC_RADIO_INCLUDE_RADIO_H_

#include <stdint.h>
#include <msprf24.h>
#include "radio_config.h"

typedef enum _radio_err_t {
    RADIO_OK,
    RADIO_EMPTY_ACK_ERR,
    RADIO_MAX_RT_ERR,
    RADIO_ERR,
} radio_err_t;

void radio_init(uint8_t channel);

#ifdef RADIO_PRIM_RX
void radio_listen(uint8_t * const out_data, uint8_t * const out_data_len);
#elif RADIO_PRIM_TX
radio_err_t radio_receive(uint8_t * const out_data, uint8_t * const out_data_len);
#endif

void radio_transmit(const uint8_t * const data, uint8_t data_len);

#endif /* SRC_RADIO_INCLUDE_RADIO_H_ */
