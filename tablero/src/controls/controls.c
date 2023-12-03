/*
 * controls.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>
#include "controls.h"
#include "controls_config.h"

static uint8_t direction_reverse = 0u;

void controls_init(void)
{
    adc10_init((STEERING_WHEEL_PIN | PEDAL_PIN));

    P1SEL &= ~REVERSE_TOGGLE_PIN;
    P1SEL2 &= ~REVERSE_TOGGLE_PIN;

    P1IES |= REVERSE_TOGGLE_PIN;
    P1IE |= REVERSE_TOGGLE_PIN;
    P1DIR &= ~REVERSE_TOGGLE_PIN;
}

uint8_t controls_get_state(uint8_t * out_speed, uint8_t * out_direction)
{
    if (ADC10_SAMPLES_READY_FG & adc10_flags)
    {
        adc10_flags &= ~ADC10_SAMPLES_READY_FG;

        *out_speed = ((direction_reverse << 7) | ((uint8_t) (adc10_samples[0] >> 3u)));

        *out_direction = ((uint8_t) (adc10_samples[1] >> 2u));

        return 1u;
    }
    else
    {
        return 0u;
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void port1_isr(void)
{
    if (REVERSE_TOGGLE_PIN & P1IFG)
    {
        P1IFG &= ~REVERSE_TOGGLE_PIN;
        direction_reverse = !direction_reverse;
    }
}
