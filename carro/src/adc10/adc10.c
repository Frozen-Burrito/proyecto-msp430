/*
 * adc10.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */
#include <msp430.h>
#include "adc10.h"

#ifdef ADC10_USE_DEDICATED_TIMER
void adc10_init(uint8_t adc_inputs, uint16_t sample_period_ms)
#else
void adc10_init(uint8_t adc_inputs)
#endif
{
    ADC10AE0 |= adc_inputs;                                     // Habilitar solo los pines usados como entradas analogicas.
    ADC10CTL1 |= (((uint16_t) ADC10_FIRST_CHANNEL) * 0x1000u);  // Empezar el muestreo desde ADC10_FIRST_CHANNEL.

    // Empezar el muestreo desde ADC10_FIRST_CHANNEL, configurar modo.
#ifdef ADC10_CHANNEL_SEQUENCE_ONE_SAMPLE
    // Una sola muestra de una secuencia de canales.
    ADC10CTL1 |= ((((uint16_t) ADC10_FIRST_CHANNEL) * 0x1000u) | CONSEQ_1);
#else
    // Una sola muestra de un solo canal.
    ADC10CTL1 |= (((uint16_t) ADC10_FIRST_CHANNEL) * 0x1000u);
#endif

    ADC10DTC1 = ADC10_SAMPLE_BUF_LEN;                           // Numero total de conversiones.
    ADC10SA = (uint16_t) &adc10_samples;                        // Inicio del buffer de conversiones del ADC.

    ADC10CTL0 |= (ADC10SHT_2 | MSC | ADC10ON | ADC10IE | ENC);

#ifdef ADC10_USE_DEDICATED_TIMER
    timer_start(ADC10_TIMER_SOURCE, ADC10_TIMER_COUNT, sample_period_ms, adc10_start_conversion);
#endif /* ADC10_USE_DEDICATED_TIMER */

    adc10_flags |= ADC10_INITIALIZED;
}

void adc10_start_conversion(void)
{
    ADC10CTL0 |= ADC10SC;
}

#pragma vector=ADC10_VECTOR
__interrupt void adc10_isr(void)
{
    adc10_flags |= ADC10_SAMPLES_READY_FG;

    // Recargar direccion de adc_samples.
    ADC10CTL0 &= ~ENC;
    ADC10SA = ((uint16_t) &adc10_samples);
    ADC10CTL0 |= ENC;

    __bic_SR_register_on_exit(CPUOFF);
}
