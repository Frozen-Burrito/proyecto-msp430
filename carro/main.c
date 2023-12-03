/**
 * Proyecto - Carro a control remoto.
 *
 */
#include <msp430.h>
#include <stdint.h>

#include "battery_mon.h"
#include "battery_mon_config.h"
#include "em.h"
#include "gps.h"
#include "motor_control.h"
#include "radio.h"
#include "timer.h"
#include "watchdog.h"

/* Motores CD */
#define VEL_INPUT_CAPTURE_PIN   (BIT2)

#define EVENT_TIMER_PERIOD_MS   (10u)

#define GPS_UART_BITRATE    (9600u)

#define RADIO_CHANNEL       (25u)

/*
 * Recepción y retransmisión del estado del carro.
 */
#define STATE_BUF_LEN           (21u)

#define LAT_INT_MSB (0u)
#define LAT_INT_LSB (1u)
#define LAT_DEC_MSB (2u)
#define LAT_DEC_XSB (3u)
#define LAT_DEC_LSB (4u)
#define LON_INT_MSB (5u)
#define LON_INT_LSB (6u)
#define LON_DEC_MSB (7u)
#define LON_DEC_XSB (8u)
#define LON_DEC_LSB (9u)
#define GPS_STAT_FG (10u)
#define VEL_RPM_MSB (11u)
#define VEL_RPM_LSB (12u)
#define BATT_MV_MSB (13u)
#define BATT_MV_LSB (14u)
#define ACCEL_X_MSB (15u)
#define ACCEL_X_LSB (16u)
#define PITCH_MSB   (17u)
#define PITCH_LSB   (18u)
#define ROLL_MSB    (19u)
#define ROLL_LSB    (20u)

/*
 * Recepcion de señal de control al carro.
 */
#define CONTROL_PAYLOAD_LEN (3u)

#define DIR_CTRL_MSB (0u)
#define DIR_CTRL_LSB (1u)
#define VEL_CTRL_VAL (2u)

#define BEGIN_BATTERY_MONITOR_SAMPLE    (BIT0)

/*
 * Declaraciones de funciones.
 */
static void input_capture_init(void);

static void state_update_gps_data(uint8_t * const buffer);

/**
 * Variables de estado global.
 */
static volatile uint8_t banderas_sistema = 0b00000000;

// El tiempo (ms) que tarda en girar 1/20 de revolucion la rueda.
static volatile uint16_t rev_fraction_period_ms = 0u;

int main(void)
{
    uint8_t state_transmit_buffer[STATE_BUF_LEN] = {};
    uint8_t control_receive_buffer[CONTROL_PAYLOAD_LEN] = {};
    uint8_t received_payload_len;

    uint16_t target_steering = 0x03FFu >> 1;
    uint8_t target_speed = 100u;

    WATCHDOG_STOP;

    DCO_LOWEST_FREQ;
    BCS_1MHZ;
    DCO_1MHZ;

    input_capture_init();
    battery_mon_init();
    gps_init(GPS_UART_BITRATE);

    radio_init(RADIO_CHANNEL);
    radio_transmit(state_transmit_buffer, STATE_BUF_LEN);

    motor_control_init();

    EM_GLOBAL_INTERRUPT_ENABLE;
    EM_ENTER_LPM0;

    while (1)
    {
        received_payload_len = radio_listen(control_receive_buffer);

        if (CONTROL_PAYLOAD_LEN == received_payload_len)
        {
            // Control de motores con payload recibida.
            target_steering = ((((uint16_t) control_receive_buffer[DIR_CTRL_MSB]) << 8u) | control_receive_buffer[DIR_CTRL_LSB]);
            target_speed = control_receive_buffer[VEL_CTRL_VAL];

            motor_control(target_steering, target_speed);

            // Obtener datos de sensores para la siguiente transmision de estado.
            uint16_t battery_mv = battery_mon_get_voltage();
            state_transmit_buffer[BATT_MV_MSB] = (uint8_t) (battery_mv >> 8u);
            state_transmit_buffer[BATT_MV_LSB] = (uint8_t) (battery_mv & 0x00FFu);

            state_transmit_buffer[VEL_RPM_MSB] = (uint8_t) (rev_fraction_period_ms >> 8u);
            state_transmit_buffer[VEL_RPM_LSB] = (uint8_t) (rev_fraction_period_ms & 0x00FFu);

            if (new_gps_data)
            {
                state_update_gps_data(state_transmit_buffer);
            }

            radio_transmit(state_transmit_buffer, STATE_BUF_LEN);
        }

        timer_execute_pending_callbacks();

        EM_ENTER_LPM0;
    }

    return 0;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer_a0_taifg_isr(void)
{
    // Esta ISR es invocada para CCR1, CCR2 y TAIFG (overflow).
    static volatile uint16_t timer_overflow_count = 0u;
    static uint8_t i = 0u;
    static uint16_t edges[2u] = {};

    switch (TA0IV)
    {
    case TA0IV_TACCR1:
        if (CCI & TA0CCTL1)
        {
            edges[i++] = TA0CCR1;

            if (2u <= i)
            {
                i = 0u;

                if (edges[0] > edges[1])
                {
                    if (0u != timer_overflow_count)
                    {
                        timer_overflow_count--;
                    }

                    rev_fraction_period_ms = (uint16_t) (((uint32_t) (edges[1] - edges[0])) + (0xFFFFu * timer_overflow_count)) / 1000u;
                }
                else
                {
                    rev_fraction_period_ms = (uint16_t) (((uint32_t) (edges[0] - edges[1])) + (0xFFFFu * timer_overflow_count)) / 1000u;
                }
            }

            timer_overflow_count = 0u;
        }
        break;
    case TA0IV_TAIFG:
        timer_overflow_count++;
        break;
    }
}

void input_capture_init(void)
{
    P1SEL |= VEL_INPUT_CAPTURE_PIN;
    P1SEL2 &= ~VEL_INPUT_CAPTURE_PIN;

    P1DIR &= ~VEL_INPUT_CAPTURE_PIN;

    TA0CCTL1 |= (CM_1 | CCIS_0 | SCS | CAP | CCIE);
}

void state_update_gps_data(uint8_t * const buffer)
{
    buffer[LAT_INT_MSB] = (lat_int >> 8u);
    buffer[LAT_INT_LSB] = lat_int;
    buffer[LAT_DEC_MSB] = (lat_dec >> 16u);
    buffer[LAT_DEC_XSB] = (lat_dec >> 8u);
    buffer[LAT_DEC_LSB] = lat_dec;
    buffer[LON_INT_MSB] = (lon_int >> 8u);
    buffer[LON_INT_LSB] = lon_int;
    buffer[LON_DEC_MSB] = (lon_dec >> 16u);
    buffer[LON_DEC_XSB] = (lon_dec >> 8u);
    buffer[LON_DEC_LSB] = lon_dec;
    buffer[GPS_STAT_FG] = (n_sat << 2u) | (gps_flags);
}
