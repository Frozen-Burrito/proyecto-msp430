/*
 * l298n.c
 *
 *  Created on: Nov 30, 2023
 *      Author: Fernando Mendoza
 */
#include "l298n.h"
#include <msp430.h>
#include "l298n_config.h"

void l298n_init(uint8_t motor_channels)
{
    uint8_t pwm_pins = 0u;

    if (MOTOR_CHANNEL_A & motor_channels)
    {
        pwm_pins |= MOTOR_A_PWM_PIN;
    }

    if (MOTOR_CHANNEL_B & motor_channels)
    {
        pwm_pins |= MOTOR_B_PWM_PIN;
    }

    timer_pwm_init(MOTOR_PWM_TIMER_SOURCE, MOTOR_PWM_PERIOD_US, pwm_pins, 0u);

#ifndef MOTOR_FORWARD_ONLY
    // Configurar el puerto 2 para controlar la direccion de los motores CD.
    P2SEL &= ~(MOTOR_IN1_PIN | MOTOR_IN2_PIN);
    P2SEL2 &= ~(MOTOR_IN1_PIN | MOTOR_IN2_PIN);

    P2OUT &= ~(MOTOR_IN1_PIN | MOTOR_IN2_PIN);
    P2DIR |= (MOTOR_IN1_PIN | MOTOR_IN2_PIN);
#endif
}

void l298n_motor_set_speed(uint8_t motor_channels, uint8_t speed, uint8_t direction)
{
    if (MOTOR_CHANNEL_A & motor_channels)
    {
        uint16_t motor_a_pwm_duty_cycle_us = speed << 2;

        if (MOTOR_PWM_PERIOD_US < motor_a_pwm_duty_cycle_us)
        {
            motor_a_pwm_duty_cycle_us = MOTOR_PWM_PERIOD_US;
        }

        timer_pwm_set_duty(MOTOR_PWM_TIMER_SOURCE, MOTOR_A_PWM_PIN, motor_a_pwm_duty_cycle_us);

#ifndef MOTOR_FORWARD_ONLY
        if (MOTOR_DIRECTION_FORWARD == direction)
        {
            P2OUT &= ~MOTOR_IN2_PIN;
            P2OUT |= MOTOR_IN1_PIN;
        }
        else if (MOTOR_DIRECTION_REVERSE == direction)
        {
            P2OUT &= ~MOTOR_IN1_PIN;
            P2OUT |= MOTOR_IN2_PIN;
        }
#endif
    }

    if (MOTOR_CHANNEL_B & motor_channels)
    {
        uint16_t motor_b_pwm_duty_cycle_us = speed << 2;

        if (MOTOR_PWM_PERIOD_US < motor_b_pwm_duty_cycle_us)
        {
            motor_b_pwm_duty_cycle_us = MOTOR_PWM_PERIOD_US;
        }

        timer_pwm_set_duty(MOTOR_PWM_TIMER_SOURCE, MOTOR_B_PWM_PIN, motor_b_pwm_duty_cycle_us);

#ifndef MOTOR_FORWARD_ONLY
        if (MOTOR_DIRECTION_FORWARD == direction)
        {
            P2OUT &= ~MOTOR_IN2_PIN;
            P2OUT |= MOTOR_IN1_PIN;
        }
        else if (MOTOR_DIRECTION_REVERSE == direction)
        {
            P2OUT &= ~MOTOR_IN1_PIN;
            P2OUT |= MOTOR_IN2_PIN;
        }
#endif
    }
}
