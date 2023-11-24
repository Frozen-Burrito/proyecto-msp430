/*
 * uart.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>

#include "uart.h"
#include "uart_config.h"

static volatile uint8_t transmit_len = 0u;
static volatile uint8_t transmit_buf[UART_TX_BUF_LEN] = {};

static volatile uint8_t receive_byte_idx = 0u;
static volatile uint8_t receive_buf[UART_RX_BUF_LEN] = {};

void uart_init(uint16_t bitrate)
{
    UCA0CTL1 |= UCSWRST;

    UCA0CTL1 |= UCSSEL_2;

    uint16_t uart_clk_divider = 1u;

    if (0u < bitrate)
    {
        uart_clk_divider = UART_CLK_FREQ_HZ / bitrate;
    }

    UCA0BR0 = ((uint8_t) uart_clk_divider);
    UCA0BR1 = ((uint8_t) uart_clk_divider >> 8u);

    P1SEL |= (UART_RX_PIN | UART_TX_PIN);
    P1SEL2 |= (UART_RX_PIN | UART_TX_PIN);

    UCA0CTL1 &= ~UCSWRST;

    uart_flags |= UART_TX_READY_FLAG;

//    IE2 |= UCA0RXIE;
}

void uart_transmit(const uint8_t * const buf, uint8_t size)
{
    if (0u == buf || 0u == size || UART_TX_BUF_LEN < size || 0u == (UART_TX_READY_FLAG & uart_flags)) return;

    uart_flags &= ~UART_TX_READY_FLAG;
    transmit_len = size;

    uint8_t i;
    for (i = 0; i < size; i++)
    {
        transmit_buf[i] = buf[i];
    }

    IE2 |= UCA0TXIE;
}

void uart_read_received(uint8_t * const out_buf, uint8_t size)
{

}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void usci_tx_isr(void)
{
    static uint8_t transmit_byte_idx = 0u;

    UCA0TXBUF = transmit_buf[transmit_byte_idx];
    transmit_byte_idx++;

    if (transmit_len <= transmit_byte_idx)
    {
        IE2 &= ~UCA0TXIE;
        transmit_byte_idx = 0u;
        uart_flags |= UART_TX_READY_FLAG;
        __bic_SR_register_on_exit(CPUOFF);
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void usci_rx_isr(void)
{
    receive_buf[receive_byte_idx] = UCA0RXBUF;
    receive_byte_idx++;

//    if (receive_byte_idx)

    if ((uart_end_sequence[0] == receive_buf[receive_byte_idx - 2]) && (uart_end_sequence[1] == receive_buf[receive_byte_idx - 1]))
    {
        receive_byte_idx = 0u;
        uart_flags |= UART_RX_READY_FLAG;
        __bic_SR_register(CPUOFF);
    }
}
