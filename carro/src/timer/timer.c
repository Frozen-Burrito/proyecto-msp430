/*
 * timer.c
 *
 *  Created on: Nov 19, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>

#include "timer.h"
#include "timer_config.h"

#define TIMER_A0_0_FLAG (0x10u)
#define TIMER_A0_1_FLAG (0x08u)

#define P2_TA1_1_MASK   (BIT1 | BIT2)
#define P2_TA1_2_MASK   (BIT4 | BIT5)

#define MAX_TIMER_COUNT (5u)

static uint8_t pending_callbacks = 0x00u;

static timer_callback_t callbacks[MAX_TIMER_COUNT] = {};
static uint16_t timer_counts_ms[MAX_TIMER_COUNT] = {};
static uint16_t timer_periods_ms[MAX_TIMER_COUNT] = {};

void timer_start(uint8_t timer_id, uint8_t count_id, uint16_t period_ms, timer_callback_t cb)
{
    if (TIMER_A0 == timer_id)
    {
        switch (count_id)
        {
        case COUNT_1:
            timer_periods_ms[0] = period_ms;
            timer_counts_ms[0] = period_ms;
            callbacks[0] = cb;
            TA0CCR0 = 1000u;
            TA0CCTL0 |= CCIE;
            break;
        case COUNT_2:
            timer_periods_ms[1] = period_ms;
            timer_counts_ms[1] = period_ms;
            callbacks[1] = cb;
            TA0CCR1 = 1000u;
            TA0CCTL1 |= CCIE;
            break;
        }

        TA0CTL |= (TASSEL_2 | MC_2 | TACLR);
    }
//    else if (TIMER_A1 == timer_id)
//    {
//        switch (count_id)
//        {
//        case COUNT_1:
//            TA1
//            break;
//        }
//    }
}

void timer_input_capture(uint8_t timer_id, uint8_t count_id, uint8_t pin)
{
    if (TIMER_A0 == timer_id)
    {
        switch (count_id)
        {
        case COUNT_1:
            TA0CCTL0 |= (CM_1 | CCIS_0 | SCS | CAP | CCIE);
            TA0CTL |= TAIE;
            break;
        case COUNT_2:
            TA0CCTL1 |= (CM_1 | CCIS_0 | SCS | CAP | CCIE);
            TA0CTL |= TAIE;
            break;
        }

        TA0CTL |= (TASSEL_2 | MC_2 | TACLR);
    }
}

void timer_execute_pending_callbacks(void)
{
    if ((TIMER_A0_0_FLAG & pending_callbacks) && (0u != callbacks[0]))
    {
        pending_callbacks &= ~TIMER_A0_0_FLAG;
        callbacks[0]();
    }

    if ((TIMER_A0_1_FLAG & pending_callbacks) && (0u != callbacks[1]))
    {
        pending_callbacks &= ~TIMER_A0_1_FLAG;
        callbacks[1]();
    }
}

void timer_pwm_init(uint8_t timer_id, uint16_t period_us, uint8_t pin_mask, uint8_t initial_duty_period_us)
{
    if (TIMER_A1 == timer_id)
    {
        TA1CTL &= ~(MC_2);

        TA1CCR0 = period_us;

        if (P2_TA1_1_MASK & pin_mask)
        {
            // Configurar funcionalidad PWM en pines seleccionados de P2.
            P2SEL |= (pin_mask & P2_TA1_1_MASK);
            P2SEL2 &= ~(pin_mask & P2_TA1_1_MASK);

            P2DIR |= (pin_mask & P2_TA1_1_MASK);

            TA1CCR1 = initial_duty_period_us;
            TA1CCTL1 |= OUTMOD_7;
        }

        if (P2_TA1_2_MASK & pin_mask)
        {
            // Configurar funcionalidad PWM en pines seleccionados de P2.
            P2SEL |= (pin_mask & P2_TA1_2_MASK);
            P2SEL2 &= ~(pin_mask & P2_TA1_2_MASK);

            P2DIR |= (pin_mask & P2_TA1_2_MASK);

            TA1CCR2 = initial_duty_period_us;
            TA1CCTL2 |= OUTMOD_7;
        }

        TA1CTL |= (TASSEL_2 | MC_1 | TACLR);
    }
}

void timer_pwm_set_duty(uint8_t timer_id, uint8_t pin_mask, uint16_t duty_period_us)
{
    if (TIMER_A1 == timer_id)
    {
        if (P2_TA1_1_MASK & pin_mask)
        {
            TA1CCR1 = duty_period_us;
        }

        if (P2_TA1_2_MASK & pin_mask)
        {
            TA1CCR2 = duty_period_us;
        }
    }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void timer_a0_ccr0_isr(void)
{
    timer_counts_ms[0]--;

    if (0u == timer_counts_ms[0])
    {
        timer_counts_ms[0] = timer_periods_ms[0];
        pending_callbacks |= TIMER_A0_0_FLAG;
        __bic_SR_register_on_exit(CPUOFF);
    }

    TA0CCR0 += 1000u;
}

//#pragma vector=TIMER0_A1_VECTOR
//__interrupt void timer_a0_taifg_isr(void)
//{
//    if (TA0IV_TACCR1 == TA0IV)
//    {
//        timer_counts_ms[1]--;
//
//        if (0u == timer_counts_ms[1])
//        {
//            timer_counts_ms[1] = timer_periods_ms[1];
//            pending_callbacks |= TIMER_A0_1_FLAG;
//            __bic_SR_register_on_exit(CPUOFF);
//        }
//
//        TA0CCR1 += 1000u;
//    }
//}
