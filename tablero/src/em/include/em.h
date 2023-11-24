/*
 * em.h
 *
 *  Created on: Nov 19, 2023
 *      Author: f3rm3
 */

#ifndef SRC_EM_INCLUDE_EM_H_
#define SRC_EM_INCLUDE_EM_H_

#include <msp430.h>

#define EM_GLOBAL_INTERRUPT_ENABLE __bis_SR_register(GIE)
#define EM_ENTER_LPM0 __bis_SR_register(CPUOFF)


#endif /* SRC_EM_INCLUDE_EM_H_ */
