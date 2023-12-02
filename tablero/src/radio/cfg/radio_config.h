/*
 * radio_config.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_RADIO_CFG_RADIO_CONFIG_H_
#define SRC_RADIO_CFG_RADIO_CONFIG_H_
#include <msprf24.h>

#define RADIO_SPEED_POWER   (RF24_SPEED_250KBPS | RF24_POWER_0DBM)
#define RADIO_ADDRESS_WIDTH ((uint8_t) 5u)

#define RADIO_MAX_RT_COUNT  (5u)
#define RADIO_RT_DELAY_US   (2000u)

#define RADIO_PIPE_0        ((uint8_t) 0u)

#define RADIO_USE_CRC

#define RADIO_PRIM_TX       (1u)    // Tablero
//#define RADIO_PRIM_RX       (1u)    // Carro

#endif /* SRC_RADIO_CFG_RADIO_CONFIG_H_ */
