/*
 * velocity_sensor.c
 *
 *  Created on: Dec 3, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>
#include "velocity_sensor.h"
#include "velocity_sensor_config.h"

void velocity_sensor_init(void)
{
    P1SEL |= VEL_INPUT_CAPTURE_PIN;
    P1SEL2 &= ~VEL_INPUT_CAPTURE_PIN;

    P1DIR &= ~VEL_INPUT_CAPTURE_PIN;

    TA0CCTL1 |= (CM_1 | CCIS_0 | SCS | CAP | CCIE);
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer_a0_taifg_isr(void)
{
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

                    rev_fraction_period_us = (((uint32_t) (edges[1] - edges[0])) + (0xFFFFu * timer_overflow_count));
                }
                else
                {
                    rev_fraction_period_us = (((uint32_t) (edges[0] - edges[1])) + (0xFFFFu * timer_overflow_count));
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
