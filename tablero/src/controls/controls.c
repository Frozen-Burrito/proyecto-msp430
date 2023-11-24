/*
 * controls.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */
#include "controls.h"
#include "controls_config.h"

void controls_init(void)
{
    adc10_init((STEERING_WHEEL_PIN | PEDAL_PIN));
}

uint8_t controls_get_state(uint8_t * out_speed, uint16_t * out_direction)
{
    if (ADC10_SAMPLES_READY_FG & adc10_flags)
    {
        adc10_flags &= ~ADC10_SAMPLES_READY_FG;

        *out_speed = ((uint8_t) (adc10_samples[0] >> 2u));

        *out_direction = (adc10_samples[1]);

        return 1u;
    }
    else
    {
        return 0u;
    }
}
