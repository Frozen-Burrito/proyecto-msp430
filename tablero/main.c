/**
 * Proyecto - Tablero de instrumentos.
 *
 */
#include <stdint.h>

#include "controls.h"
#include "em.h"
#include "radio.h"
#include "uart.h"
#include "watchdog.h"

#define UART_BITRATE    ((uint16_t) 9600u)

#define RADIO_CHANNEL   ((uint8_t) 25u)

/*
 * Recepci�n y retransmisi�n del estado del carro.
 */
#define STATE_BUF_MAX_PAYLOAD_LEN   (21u)
#define STATE_BUF_MAX_LEN           (STATE_BUF_MAX_PAYLOAD_LEN + 3u)

#define STATE_BUF_SYS_STATUS_BYTE   (0u)

/*
 * Transmisi�n de se�al de control al carro.
 */
#define CONTROL_PAYLOAD_LEN (2u)

#define DIR_CTRL_VAL (0u)
#define VEL_CTRL_VAL (1u)

int main(void)
{
    uint8_t control_tx_buf[CONTROL_PAYLOAD_LEN] = {};
    uint8_t state_tx_rx_buf[STATE_BUF_MAX_LEN] = {};
    uint8_t state_rx_payload_len;
    radio_err_t radio_status;

    uint8_t steering_pos;
    uint8_t pedal_pos;

    WATCHDOG_STOP;

    DCO_LOWEST_FREQ;
    BCS_1MHZ;
    DCO_1MHZ;

    controls_init();

    uart_init(UART_BITRATE);

    radio_init(RADIO_CHANNEL);

    EM_GLOBAL_INTERRUPT_ENABLE;
    EM_ENTER_LPM0;

    while (1)
    {
        if (controls_get_state(&pedal_pos, &steering_pos))
        {
            control_tx_buf[DIR_CTRL_VAL] = steering_pos;
            control_tx_buf[VEL_CTRL_VAL] = pedal_pos;

            radio_transmit(control_tx_buf, CONTROL_PAYLOAD_LEN);

            LPM4;

            radio_status = radio_receive(&(state_tx_rx_buf[1]), &state_rx_payload_len);

            state_tx_rx_buf[STATE_BUF_SYS_STATUS_BYTE] = radio_status;
            state_tx_rx_buf[STATE_BUF_MAX_LEN - 2u] = '\r';
            state_tx_rx_buf[STATE_BUF_MAX_LEN - 1u] = '\n';

            uart_transmit(state_tx_rx_buf, STATE_BUF_MAX_LEN);
        }

        timer_execute_pending_callbacks();

        EM_ENTER_LPM0;
    }

    return 0;
}
