/**
 * Proyecto - Carro a control remoto.
 *
 * Conexiones:
 * P2.1 - ENA, PWM para velocidad del motor CD.
 * P2.0 - IN1 Motor CD
 * P2.2 - IN2 Motor CD
 * P2.4 - P2.7 -> In1 - In4 de motor a pasos para controlar direccion.
 *
 * Mantiene P1.1 y P1.2 libres para UART.
 * Mantiene P1.4, P1.5, P1.6 y P1.7 libres para SPI/I2C.
 *
 * Timer A1 dedicado a PWM, Timer A0 dedicado a tiempos internos.
 *
 * Pines restantes:
 * P1.0, P1.3, P2.3
 */
#include <msp430.h>

#define PIN_PWM_MOTOR_CD        (BIT1)
#define PINES_DIR_MOTOR_CD      (BIT0 | BIT2)
#define PINES_MOTOR_DIRECCION   (BIT4 | BIT5 | BIT6 | BIT7)

#define PERIODO_VAL_STEPPER_US  (2400u)

// 1 vuelta = 4 valores * 512 pasos = 2048 valores.
// Media vuelta = 1024 valores.
static volatile unsigned short posicion_deseada = 512u;

// Velocidad del motor, expresada en duty cycle (0% - 100%).
static volatile unsigned char duty_cycle_velocidad = 0u;

// Timer A1 CCR1/CCR2/TAIFG ISR
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TimerA0_ccr1_isr(void)
{
    static const unsigned char secuencia_stepper[] = {0xC0, 0x60, 0x30, 0x90};
    static unsigned char i = 0u;

    static short posicion_actual = 512u;

    switch (TA0IV)
    {
    case TA0IV_TACCR1:
        P2OUT &= ~PINES_MOTOR_DIRECCION;

        if (posicion_actual < posicion_deseada)
        {
            i++;
            posicion_actual++;

            if (i >= 4u)
            {
                i = 0u;
            }

            P2OUT |= secuencia_stepper[i];
        }
        else if (posicion_actual > posicion_deseada)
        {
            i--;
            posicion_actual--;

            if (i >= 4u)
            {
                i = 3u;
            }

            P2OUT |= secuencia_stepper[i];
        }

        TA0CCR1 += PERIODO_VAL_STEPPER_US;
        break;
    }
}

int main(void)
{
    // Detener el watchdog timer.
    WDTCTL = WDTPW | WDTHOLD;

    // Configurar el puerto 2 para controlar el motor a pasos y el motor CD.
    P2SEL &= ~(PINES_MOTOR_DIRECCION | PINES_DIR_MOTOR_CD);
    P2SEL2 &= ~(PINES_MOTOR_DIRECCION | PINES_DIR_MOTOR_CD | PIN_PWM_MOTOR_CD);

    P2SEL |= (PIN_PWM_MOTOR_CD); // Usar P2.1 como salida PWM.

    P2DIR |= (PINES_MOTOR_DIRECCION | PINES_DIR_MOTOR_CD | PIN_PWM_MOTOR_CD);

    // El motor de CD siempre esta avanzando, configurar estado de pines desde el
    // inicio.
    P2OUT &= ~PINES_DIR_MOTOR_CD;
    P2OUT |= BIT0;

    // Timer A0 para tiempos internos, como el del motor a pasos.
    TA0CTL |= TASSEL_2 | MC_2;
    TA0CCTL1 |= CCIE;
    TA0CCR1 = TA0R + PERIODO_VAL_STEPPER_US;

    // Timer A1 para PWM de los motores CD.
    TA1CTL |= TASSEL_2 | MC_1 | TACLR;
    TA1CCR0 = 1000u - 1u;
    TA1CCR1 = duty_cycle_velocidad;
    TA1CCTL1 |= OUTMOD_7;

    __bis_SR_register(GIE);

    while (1);

    return 0;
}
