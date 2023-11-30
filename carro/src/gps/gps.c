/*
 * uart.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>
#include "gps.h"
#include "gps_config.h"

void gps_init(uint16_t bitrate)
{
    UCA0CTL1 |= UCSWRST;

    UCA0CTL1 |= UCSSEL_2;

    uint16_t uart_clk_divider = 1u;

    if (0u < bitrate)
    {
        uart_clk_divider = GPS_UART_CLK_FREQ_HZ / bitrate;
    }

    UCA0BR0 = ((uint8_t) uart_clk_divider);
    UCA0BR1 = ((uint8_t) uart_clk_divider >> 8u);

    P1SEL |= (1u << GPS_UART_TX_PIN);
    P1SEL2 |= (1u << GPS_UART_TX_PIN);

    UCA0CTL1 &= ~UCSWRST;

   IE2 |= UCA0RXIE;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void usci_rx_isr(void)
{
    static uint8_t parser_state = 0u;
    static uint8_t n = 0u;
    static uint8_t n_g = 0u;

    uint8_t c = UCA0RXBUF;

    switch (parser_state)
    {
    case 0:
        if ('$' == c)
        {
            n = 0u;
            n_g = 0u;
            parser_state = 1u;
        }
        break;
    case 1:
        n++;
        if ('G' == c) n_g++;

        if (3u == n_g)
        {
            new_gps_data = 0u;
            lat_int = 0u;
            lat_dec = 0u;
            lon_int = 0u;
            lon_dec = 0u;
            n_sat = 0u;
            gps_flags = 0x00;
            parser_state = 2u;
        }
        else if (5u <= n) parser_state = 0u;
        break;
    case 2:
        if (',' == c) parser_state = 3u;
        break;
    case 3:
        if (',' == c) parser_state = 4u;
        break;
    case 4:
        if ('.' == c) parser_state = 5u;
        else lat_int = (lat_int * 10u) + (c - '0');
        break;
    case 5:
        if (',' == c) parser_state = 6u;
        else lat_dec = (lat_dec * 10u) + (c - '0');
        break;
    case 6:
        if ('N' == c || 'S' == c)
        {
            gps_flags |= (('S' == c) & GPS_LAT_DIR_FLAG);
            parser_state = 7u;
        }
        break;
    case 7:
        if (',' == c) parser_state = 8u;
        break;
    case 8:
        if ('.' == c) parser_state = 9u;
        else lon_int = (lon_int * 10u) + (c - '0');
        break;
    case 9:
        if (',' == c) parser_state = 10u;
        else lon_dec = (lon_dec * 10u) + (c - '0');
        break;
    case 10:
        if ('W' == c || 'E' == c)
        {
            gps_flags |= ((('E' == c) << 1) & GPS_LON_DIR_FLAG);
            parser_state = 11u;
        }
        break;
    case 11:
        if (',' == c) parser_state = 12u;
        break;
    case 12:
        if (',' == c) parser_state = 13u;
        break;
    case 13:
        if (',' == c)
        {
            parser_state = 0u;
            new_gps_data = 1u;
            __bic_SR_register_on_exit(CPUOFF);
        }
        else n_sat = (n_sat * 10u) + (c - '0');
        break;
    }
}
