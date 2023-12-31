/**
 * Proyecto - Carro a control remoto.
 *
 */
#include <msp430.h>
#include <stdint.h>

#include "battery_mon.h"
#include "em.h"
#include "gps.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "radio.h"
#include "timer.h"
#include "velocity_sensor.h"
#include "watchdog.h"

#ifdef MPU6050_HARDWARE_I2C
#include "i2c_bus.h"
#else
#include "sw_i2c_bus.h"
#endif

#define EVENT_TIMER_PERIOD_MS           ((uint16_t) 10u)
#define MPU6050_SAMPLE_PERIOD_COUNTS    ((uint16_t) 5u)
#define BATTERY_SAMPLE_PERIOD_COUNTS    ((uint16_t) 500u)

#define GPS_UART_BITRATE    (9600u)

#define RADIO_CHANNEL       (25u)

/*
 * Recepci�n y retransmisi�n del estado del carro.
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
#define REV_MS_MSB  (11u)
#define REV_MS_LSB  (12u)
#define BATT_MV_MSB (13u)
#define BATT_MV_LSB (14u)
#define ACCEL_X_MSB (15u)
#define ACCEL_X_LSB (16u)
#define PITCH_MSB   (17u)
#define PITCH_LSB   (18u)
#define ROLL_MSB    (19u)
#define ROLL_LSB    (20u)

/*
 * Recepcion de se�al de control al carro.
 */
#define CONTROL_PAYLOAD_LEN (2u)

#define DIR_CTRL_VAL (0u)
#define VEL_CTRL_VAL (1u)

static uint8_t state_transmit_buffer[STATE_BUF_LEN] = {};

static uint8_t are_motors_enabled = 0u;

static void state_update_gps_data(uint8_t * const buffer);
static void timer_event_callback(void);

int main(void)
{
    uint8_t control_receive_buffer[CONTROL_PAYLOAD_LEN] = {};
    uint8_t received_payload_len;
    static volatile uint8_t mpu6050_has_error;

    uint8_t target_steering = 0xFFu >> 1;
    uint8_t target_speed = 0u;

    WATCHDOG_STOP;

    DCO_LOWEST_FREQ;
    BCS_1MHZ;
    DCO_1MHZ;

    velocity_sensor_init();
    battery_mon_init();
    gps_init(GPS_UART_BITRATE);

    i2c_master_init();
    mpu6050_has_error = mpu6050_init();

    if (0u == mpu6050_has_error)
    {
        mpu6050_interrupt_en();
    }

    timer_start(TIMER_A0, COUNT_1, EVENT_TIMER_PERIOD_MS, timer_event_callback);

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
            target_steering = control_receive_buffer[DIR_CTRL_VAL];
            target_speed = control_receive_buffer[VEL_CTRL_VAL];
            are_motors_enabled = (0u != target_speed);

            motor_control(target_steering, target_speed);

            // Obtener datos de sensores para la siguiente transmision de estado.
            uint16_t battery_mv = battery_mon_get_voltage();
            state_transmit_buffer[BATT_MV_MSB] = (uint8_t) (battery_mv >> 8u);
            state_transmit_buffer[BATT_MV_LSB] = (uint8_t) (battery_mv & 0x00FFu);

            uint16_t rev_fraction_period_ms = (uint16_t) (rev_fraction_period_us / 1000u);
            state_transmit_buffer[REV_MS_MSB] = (uint8_t) (rev_fraction_period_ms >> 8u);
            state_transmit_buffer[REV_MS_LSB] = (uint8_t) (rev_fraction_period_ms & 0x00FFu);

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

void timer_event_callback(void)
{
    static uint16_t counts_before_mpu6050_sample = MPU6050_SAMPLE_PERIOD_COUNTS;
    static uint16_t counts_before_battery_sample = BATTERY_SAMPLE_PERIOD_COUNTS;
    int16_t acce_x;
    int16_t pitch;
    int16_t roll;

    if (0u == counts_before_battery_sample)
    {
        counts_before_battery_sample = BATTERY_SAMPLE_PERIOD_COUNTS;
        battery_mon_sample(are_motors_enabled);
    }
    else
    {
        --counts_before_battery_sample;
    }

    if (0u == counts_before_mpu6050_sample)
    {
        counts_before_mpu6050_sample = MPU6050_SAMPLE_PERIOD_COUNTS;
        mpu6050_accel_pitch_roll(&acce_x, &pitch, &roll);

        state_transmit_buffer[ACCEL_X_MSB] = (acce_x >> 8u);
        state_transmit_buffer[ACCEL_X_LSB] = acce_x;
        state_transmit_buffer[PITCH_MSB] = (pitch >> 8u);
        state_transmit_buffer[PITCH_LSB] = pitch;
        state_transmit_buffer[ROLL_MSB] = (roll >> 8u);
        state_transmit_buffer[ROLL_LSB] = roll;
    }
    else
    {
        --counts_before_mpu6050_sample;
    }
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
