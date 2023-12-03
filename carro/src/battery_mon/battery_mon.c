/*
 * battery_mon.c
 *
 *  Created on: Nov 28, 2023
 *      Author: Fernando Mendoza
 */
#include "battery_mon.h"
#include <msp430.h>

#include "battery_mon_config.h"

void battery_mon_init(void)
{
    adc10_init(BATTERY_ADC_PIN, BATTERY_MON_SAMPLE_PERIOD_MS);
}

uint16_t battery_mon_get_voltage(void)
{
    uint16_t new_battery_mv = ((((uint32_t) adc10_samples[0u]) * BATTERY_ADC_MAX_MV) / 1023u) << 1u;

    if (BATTERY_MIN_MV > new_battery_mv)
    {
        new_battery_mv = BATTERY_MIN_MV;
    }
    else if (BATTERY_MAX_MV < new_battery_mv)
    {
        new_battery_mv = BATTERY_MAX_MV;
    }

    return new_battery_mv;
}
