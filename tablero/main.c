/**
 * Proyecto - Tablero de instrumentos.
 *
 * Conexiones:
 * P1.1 -> UART RXD.
 * P1.2 -> UART TXD.
 * P1.3 (A3) -> Potenciometro de control de direccion (volante).
 * P1.4 (A4) -> Potenciometro de control de velocidad (pedal).
 * P1.5 - P1.7 -> SPI.
 */
#include <stdint.h>

#include "timer.h"
#include "controls.h"
#include "em.h"
#include "radio.h"
#include "uart.h"
#include "watchdog.h"

#define UART_BITRATE    ((uint16_t) 9600u)

#define RADIO_CHANNEL   ((uint8_t) 25u)

/*
 * Recepción y retransmisión del estado del carro.
 */
#define STATE_BUF_MAX_LEN   (24u)

/*
 * Transmisión de señal de control al carro.
 */
#define CONTROL_PAYLOAD_LEN (3u)

#define DIR_CTRL_MSB (0u)
#define DIR_CTRL_LSB (1u)
#define VEL_CTRL_VAL (2u)

int main(void)
{
    uint8_t control_buf[CONTROL_PAYLOAD_LEN] = {};
    uint8_t state_buf[STATE_BUF_MAX_LEN] = {};

    uint16_t steering_pos;
    uint8_t pedal_pos;
    uint8_t radio_rx_payload_len;

    WATCHDOG_STOP;

    DCO_LOWEST_FREQ;
    BCS_1MHZ;
    DCO_1MHZ;

    controls_init();

    uart_init(UART_BITRATE);

    radio_init(RADIO_CHANNEL);

    EM_GLOBAL_INTERRUPT_ENABLE;

    while (1)
    {
        if (controls_get_state(&pedal_pos, &steering_pos))
        {
            control_buf[DIR_CTRL_MSB] = ((uint8_t) (steering_pos >> 8u));
            control_buf[DIR_CTRL_LSB] = ((uint8_t) (steering_pos & 0xFFu));
            control_buf[VEL_CTRL_VAL] = pedal_pos;

            radio_transmit(control_buf, CONTROL_PAYLOAD_LEN);
        }

        if (radio_receive(state_buf, &radio_rx_payload_len))
        {
            uart_transmit(state_buf, radio_rx_payload_len);
        }

        timer_execute_pending_callbacks();

        EM_ENTER_LPM0;
    }

    return 0;
}
