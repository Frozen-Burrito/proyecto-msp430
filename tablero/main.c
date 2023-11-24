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
#include <msp430.h>
#include <stdint.h>

#include "timer.h"
#include "controls.h"
#include "em.h"
#include "msprf24.h"
#include "uart.h"
#include "watchdog.h"

#define UART_BITRATE    ((uint16_t) 9600u)

#define NRF24_PIPE_0        (0u)
#define NRF24_RF_CHANNEL    (25u)
#define NRF24_ADDR_WIDTH    (4u)

/*
 * Recepción y retransmisión del estado del carro.
 */
#define STATE_PAYLOAD_LEN   (20u)

/*
 * Transmisión de señal de control al carro.
 */
#define CONTROL_PAYLOAD_LEN (3u)

#define DIR_CTRL_MSB (0u)
#define DIR_CTRL_LSB (1u)
#define VEL_CTRL_VAL (2u)

#define CTRL_TRANSMIT_INTERVAL_MS   ((uint8_t) 100u)

/*
 * Variables globales.
 */
static volatile uint8_t flags_estado_sistema = 0x00;

static uint8_t control_buf[CONTROL_PAYLOAD_LEN] = {};
static uint8_t state_buf[STATE_PAYLOAD_LEN] = {};

static const uint8_t rf_tx_rx_addr[NRF24_ADDR_WIDTH] = { 0x90u, 0xF0u, 0x58u, 0x85u };

int main(void)
{
    uint16_t steering_pos;
    uint8_t pedal_pos;

    // Detener el watchdog timer.
    WATCHDOG_STOP;

    spi_init();

    controls_init();

    uart_init(UART_BITRATE);

    msprf24_open_pipe(NRF24_PIPE_0, 0u);
    msprf24_set_pipe_packetsize(NRF24_PIPE_0, CONTROL_PAYLOAD_LEN);

    rf_channel = NRF24_RF_CHANNEL;
    msprf24_set_channel();

    rf_addr_width = NRF24_ADDR_WIDTH;
    msprf24_set_address_width();

    w_tx_addr(rf_tx_rx_addr);

    EM_GLOBAL_INTERRUPT_ENABLE;

    while (1)
    {
        if (controls_get_state(&pedal_pos, &steering_pos))
        {
            control_buf[DIR_CTRL_MSB] = ((uint8_t) (steering_pos >> 8u));
            control_buf[DIR_CTRL_LSB] = ((uint8_t) (steering_pos & 0xFFu));
            control_buf[VEL_CTRL_VAL] = pedal_pos;

            //TODO: Enviar buffer de control por RF.
            w_tx_payload(CONTROL_PAYLOAD_LEN, control_buf);

            msprf24_activate_tx();
        }

        msprf24_activate_rx();

        if (msprf24_rx_pending())
        {
            msprf24_get_irq_reason();

            if (RF24_IRQ_RX == rf_irq)
            {
                uint8_t rx_payload_len = r_rx_peek_payload_size();

                r_rx_payload(rx_payload_len, state_buf);

                msprf24_irq_clear(RF24_IRQ_RX);

                uart_transmit(state_buf, rx_payload_len);
            }
        }

        timer_execute_pending_callbacks();

        EM_ENTER_LPM0;
    }

    return 0;
}
