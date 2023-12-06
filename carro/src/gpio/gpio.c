/*
 * gpio.c
 *
 *  Created on: Nov 21, 2023
 *      Author: Fernando Mendoza
 */
#include "gpio.h"

#define PORT1_MASK  ((uint16_t) 0x00FFu)
#define PORT2_MASK  ((uint16_t) 0xFF00u)

void gpio_alloc_isr(uint8_t port, uint8_t pin, uint8_t isr_flags)
{
    // Px.0 - Px.7
    if (7u < pin) return;

    uint8_t pin_mask = (1u << pin);

    isr_flags &= ~pin_mask;

    switch (port)
    {
    case GPIO_PORT1:
        P1IE |= pin_mask;
        P1IES &= ~pin_mask;
        P1IES |= ((GPIO_ISR_H_TO_L_FLAG & isr_flags) << pin);
        P1SEL &= ~pin_mask;
        P1SEL2 &= ~pin_mask;
        P1REN &= ~pin_mask;
        P1REN |= ((GPIO_ISR_REN_FLAG & isr_flags) << pin);
        P1OUT &= ~pin_mask;
        P1OUT |= ((GPIO_ISR_H_TO_L_FLAG & isr_flags) << pin);
        P1DIR &= ~pin_mask;
        break;
    case GPIO_PORT2:
        P2IE |= pin_mask;
        P2IES &= ~pin_mask;
        P2IES |= ((GPIO_ISR_H_TO_L_FLAG & isr_flags) << pin);
        P2SEL &= ~pin_mask;
        P2SEL2 &= ~pin_mask;
        P2REN &= ~pin_mask;
        P2REN |= ((GPIO_ISR_REN_FLAG & isr_flags) << pin);
        P2OUT &= ~pin_mask;
        P2OUT |= ((GPIO_ISR_H_TO_L_FLAG & isr_flags) << pin);
        P2DIR &= ~pin_mask;
        break;
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void port1_isr(void)
{
    isr_flags &= ~PORT1_MASK;
    isr_flags |= (P1IFG & PORT1_MASK);
    P1IFG = 0x00u;
}
//
//#pragma vector=PORT2_VECTOR
//__interrupt void port2_isr(void)
//{
//    isr_flags &= ~PORT2_MASK;
//    isr_flags |= (((uint16_t) P2IFG) << 8u) & PORT2_MASK;
//    P2IFG = 0x00u;
//}
