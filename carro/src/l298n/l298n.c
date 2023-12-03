/*
 * l298n.c
 *
 *  Created on: Nov 30, 2023
 *      Author: Fernando Mendoza
 */
#include "l298n.h"
#include <msp430.h>
#include "l298n_config.h"

#define MOTOR_MIN_DUTY_CYCLE_US     ((MOTOR_PWM_PERIOD_US * MOTOR_CUTOFF_SPEED / MOTOR_MAX_SPEED))
#define MOTOR_DUTY_CYCLE_RANGE_US   (MOTOR_PWM_PERIOD_US - MOTOR_MIN_DUTY_CYCLE_US)

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
    // Calcular velocidad, ajustada para direccion de giro y duty cycle de cutoff.
    uint16_t motor_pwm_duty_cycle_us = (((uint16_t) speed) * MOTOR_DUTY_CYCLE_RANGE_US) / MOTOR_MAX_SPEED;

    if (0u < speed)
    {
        motor_pwm_duty_cycle_us += MOTOR_MIN_DUTY_CYCLE_US;
    }
    if (MOTOR_PWM_PERIOD_US < motor_pwm_duty_cycle_us)
    {
        motor_pwm_duty_cycle_us = MOTOR_PWM_PERIOD_US;
    }

    // Aplicar nuevo duty cycle a motores seleccionados.
    if (MOTOR_CHANNEL_A & motor_channels)
    {
        timer_pwm_set_duty(MOTOR_PWM_TIMER_SOURCE, MOTOR_A_PWM_PIN, motor_pwm_duty_cycle_us);

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
        timer_pwm_set_duty(MOTOR_PWM_TIMER_SOURCE, MOTOR_B_PWM_PIN, motor_pwm_duty_cycle_us);

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
