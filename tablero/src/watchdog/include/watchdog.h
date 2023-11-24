/*
 * watchdog.h
 *
 *  Created on: Nov 19, 2023
 *      Author: f3rm3
 */

#ifndef SRC_WATCHDOG_INCLUDE_WATCHDOG_H_
#define SRC_WATCHDOG_INCLUDE_WATCHDOG_H_

#include <msp430.h>

#define WATCHDOG_STOP WDTCTL = WDTPW | WDTHOLD

#endif /* SRC_WATCHDOG_INCLUDE_WATCHDOG_H_ */
