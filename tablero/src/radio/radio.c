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

static const uint8_t rf_tx_rx_addr[RADIO_ADDRESS_WIDTH] = { 0x90u, 0xF0u, 0x58u, 0x85u, 0x22u };

void radio_init(uint8_t channel)
{
    /* Initial values for nRF24L01+ library config variables */
#ifdef RADIO_USE_CRC
    rf_crc             = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
#endif
    rf_addr_width      = RADIO_ADDRESS_WIDTH;
    rf_speed_power     = RADIO_SPEED_POWER;
    rf_channel         = channel;

    msprf24_init();

    // Setup auto-ack with dynamic payload, to reduce load from role-swapping.
    msprf24_enable_feature(RF24_EN_ACK_PAY | RF24_EN_DPL);
    msprf24_open_pipe(RADIO_PIPE_0, NRF24_PIPE_EN_AUTO_ACKNOWLEDGE);
    msprf24_set_pipe_packetsize(RADIO_PIPE_0, NRF24_PIPE_EN_DYN_PLD);

    msprf24_set_retransmit_count(RADIO_MAX_RT_COUNT);
    msprf24_set_retransmit_delay(RADIO_RT_DELAY_US);

    w_tx_addr((uint8_t *) rf_tx_rx_addr);
    w_rx_addr(RADIO_PIPE_0, (uint8_t *) rf_tx_rx_addr);

    msprf24_activate_rx();
}

#ifdef RADIO_PRIM_RX
void radio_listen(uint8_t * const out_data, uint8_t * const out_data_len)
{
    // Await for IRQ to be activated.
    if (rf_irq & RF24_IRQ_FLAGGED)
    {
        rf_irq &= ~RF24_IRQ_FLAGGED;
        msprf24_get_irq_reason();
    }

    if ((RF24_IRQ_RX & rf_irq) | msprf24_rx_pending())
    {
        *out_data_len = r_rx_peek_payload_size();
        r_rx_payload(*out_data_len, out_data);
        msprf24_irq_clear(RF24_IRQ_RX);
    }

    if (RF24_IRQ_TX & rf_irq)
    {
        msprf24_irq_clear(RF24_IRQ_TX);
    }
}
#endif

#ifdef RADIO_PRIM_TX
radio_err_t radio_receive(uint8_t * const out_data, uint8_t * const out_data_len)
{
    // Check transmission result
    if (RF24_IRQ_FLAGGED & rf_irq)
    {
        rf_irq &= ~RF24_IRQ_FLAGGED;
        msprf24_get_irq_reason();
    }

    if (RF24_IRQ_TX & rf_irq)
    {
        if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending())
        {
            // Received an acknowledgement with a payload.
            *out_data_len = r_rx_peek_payload_size();
            r_rx_payload(*out_data_len, out_data);
            msprf24_irq_clear(RF24_IRQ_RX | RF24_IRQ_TX);

            return RADIO_OK;
        }
        else
        {
            // Received ACK, but no payload.
            msprf24_irq_clear(RF24_IRQ_TX);
            return RADIO_EMPTY_ACK_ERR;
        }
    }
    else if (RF24_IRQ_TXFAILED & rf_irq)
    {
        // TX failed.
        msprf24_irq_clear(RF24_IRQ_TXFAILED);
        return RADIO_MAX_RT_ERR;
    }

    return RADIO_ERR;
}
#endif

void radio_transmit(const uint8_t * const data, uint8_t data_len)
{
#ifdef RADIO_PRIM_TX
    w_tx_payload(data_len, (uint8_t *) data);

    msprf24_activate_tx();
#endif

#ifdef RADIO_PRIM_RX
    w_ack_payload(RADIO_PIPE_0, data_len, data);
#endif
}
