/*
 * radio.c
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>
#include "radio.h"
#include "radio_config.h"

#define NRF24_PIPE_EN_DYN_PLD           (0u)
#define NRF24_PIPE_EN_AUTO_ACKNOWLEDGE  (1u)

void radio_init(uint8_t channel)
{
    /* Initial values for nRF24L01+ library config variables */
#ifdef RADIO_USE_CRC
    rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
#endif

    rf_addr_width      = RADIO_ADDRESS_WIDTH;
    rf_speed_power     = RADIO_SPEED_POWER;
    rf_channel         = channel;

    msprf24_init();

    // Setup auto-ack with dynamic payload, to reduce load from role-swapping.
    msprf24_enable_feature(RF24_EN_ACK_PAY | RF24_EN_DPL);
    msprf24_set_pipe_packetsize(RADIO_PIPE_0, NRF24_PIPE_EN_DYN_PLD);
    msprf24_open_pipe(RADIO_PIPE_0, NRF24_PIPE_EN_AUTO_ACKNOWLEDGE);

    msprf24_set_retransmit_count(RADIO_MAX_RT_COUNT);
    msprf24_set_retransmit_delay(RADIO_RT_DELAY_US);

    w_tx_addr(rf_tx_rx_addr);
    w_rx_addr(RADIO_PIPE_0, rf_tx_rx_addr);

    msprf24_standby();
}

uint8_t radio_receive(uint8_t * const out_data, uint8_t * const out_data_len)
{
#ifdef RADIO_PRIM_RX
    if (rf_irq & RF24_IRQ_FLAGGED)
    {
        rf_irq &= ~RF24_IRQ_FLAGGED;
        msprf24_get_irq_reason();
    }
#endif

    if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending())
    {
        *out_data_len = r_rx_peek_payload_size();

        r_rx_payload(*out_data_len, out_data);
        msprf24_irq_clear(RF24_IRQ_RX);

        return 1u;
    }
    else
    {
        return 0u;
    }
}

uint8_t radio_transmit(const uint8_t * const data, uint8_t data_len)
{
#ifdef RADIO_PRIM_TX
    w_tx_payload(data_len, data);

    msprf24_activate_tx();

    // Check transmission result
    if (RF24_IRQ_FLAGGED & rf_irq)
    {
        rf_irq &= ~RF24_IRQ_FLAGGED;
        msprf24_get_irq_reason();
    }

    return ((RF24_IRQ_TX & rf_irq) && (0u == (RF24_IRQ_TXFAILED & rf_irq));
#endif

#ifdef RADIO_PRIM_RX
    w_ack_payload(RADIO_PIPE_0, data_len, data);

    return 1u;
#endif
}
