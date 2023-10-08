/**
 * Proyecto - Tablero de instrumentos.
 *
 * Conexiones:
 * P1.0 (A0) -> Potenciometro de control de direccion (volante).
 * P1.3 (A3) -> Potenciometro de control de velocidad (pedal).
 *
 * Mantiene P1.1 y P1.2 libres para UART.
 * Mantiene P1.4 - P1.7 libres para SPI.
 */
#include <msp430.h>

#define PIN_VOLANTE             (BIT0)
#define PIN_PEDAL               (BIT3)
#define CANAL_ADC_MAS_ALTO      (INCH_3)
#define PINES_ADC_IN            (PIN_VOLANTE | PIN_PEDAL)

#define NUM_MUESTRAS_ADC        (4u)
#define PERIODO_MUESTRA_ADC_US  (10000u)

static volatile short adc_samples[NUM_MUESTRAS_ADC] = {};

// 1 vuelta = 4 valores * 512 pasos = 2048 valores
// Media vuelta = 1024 valores
static volatile unsigned short posicion_deseada = 512u;

// Velocidad de los motores CD, expresada en duty cycle (0% - 100%).
static volatile unsigned char duty_cycle_velocidad = 0u;

// ISR para las conversiones del ADC10.
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    // Leer valores de los dos potenciometros desde ADC_samples.
    duty_cycle_velocidad = (((unsigned long)adc_samples[0]) * 100u) / 1023u;
    posicion_deseada = adc_samples[1];

    // Recargar direccion de adc_samples
    ADC10CTL0 &= ~ENC;
    ADC10SA = (unsigned short) &adc_samples;
    ADC10CTL0 |= ENC;
}

// Timer 0 A0 ISR
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0_ccr0_isr(void)
{
    // Iniciar la siguiente conversion.
    ADC10CTL0 |= ADC10SC;
    TA0CCR0 += PERIODO_MUESTRA_ADC_US;
}

int main(void)
{
    // Detener el watchdog timer.
    WDTCTL = WDTPW | WDTHOLD;

    // Configurar el ADC en modo de varios canales - una muestra.
    ADC10AE0 |= PINES_ADC_IN;                       // Habilitar solo los pines usados como entradas analogicas.
    ADC10CTL1 |= (CANAL_ADC_MAS_ALTO | CONSEQ_1);   // Secuencia de canales, desde el mas alto hasta A0.

    ADC10DTC1 = NUM_MUESTRAS_ADC;                   // Numero total de conversiones.
    ADC10SA = (unsigned short) &adc_samples;        // Inicio del buffer de conversiones del ADC.

    ADC10CTL0 |= (ADC10SHT_2 | MSC | ADC10ON | ADC10IE | ENC);

    // Timer A0 para muestras del ADC (1 MHz, modo continuo).
    TA0CTL |= TASSEL_2 | MC_2;
    TA0CCTL0 |= CCIE;
    TA0CCR0 = TA0R + PERIODO_MUESTRA_ADC_US;

    __bis_SR_register(GIE);

    while (1);

    return 0;
}
