/*
 * gpio.h
 *
 *  Created on: Nov 21, 2023
 *      Author: Fernando Mendoza Velasco
 */

#ifndef SRC_GPIO_INCLUDE_GPIO_H_
#define SRC_GPIO_INCLUDE_GPIO_H_
#include <msp430.h>
#include <stdint.h>

#define GPIO_PORT1              ((uint8_t) 1u)
#define GPIO_PORT2              ((uint8_t) 2u)

#define GPIO_ISR_H_TO_L_FLAG    ((uint8_t) 0x01u)
#define GPIO_ISR_REN_FLAG       ((uint8_t) 0x02u)

uint16_t isr_flags;

void gpio_alloc_isr(uint8_t port, uint8_t pin, uint8_t isr_flags);

#endif /* SRC_GPIO_INCLUDE_GPIO_H_ */
