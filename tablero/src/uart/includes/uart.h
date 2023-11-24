/*
 * uart.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_UART_INCLUDES_UART_H_
#define SRC_UART_INCLUDES_UART_H_
#include <stdint.h>

#define UART_TX_READY_FLAG  (BIT0)
#define UART_RX_READY_FLAG  (BIT1)

uint8_t uart_flags;

void uart_init(uint16_t bitrate);

void uart_transmit(const uint8_t * const buf, uint8_t size);

void uart_read_received(uint8_t * const out_buf, uint8_t size);

#endif /* SRC_UART_INCLUDES_UART_H_ */
