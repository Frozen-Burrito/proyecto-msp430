/*
 * uart_config.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_UART_CFG_UART_CONFIG_H_
#define SRC_UART_CFG_UART_CONFIG_H_

#define UART_CLK_FREQ_HZ        ((uint32_t) 1000000)

#define UART_RX_PIN             (BIT1)
#define UART_TX_PIN             (BIT2)

#define UART_BITRATE_DIVIDER    ((uint16_t) 104u)

#define UART_TX_BUF_LEN         ((uint8_t) 3u)
#define UART_RX_BUF_LEN         ((uint8_t) 16u)

const uint8_t * const uart_end_sequence = "\r\n";

#endif /* SRC_UART_CFG_UART_CONFIG_H_ */
