/*
 * motor_control.c
 *
 *  Created on: Dec 1, 2023
 *      Author: Fernando Mendoza
 */
#include "motor_control.h"

void motor_control_init(void)
{
    l298n_init(MOTOR_CHANNEL_A | MOTOR_CHANNEL_B);
}

void motor_control(uint16_t steering, uint8_t speed)
{
    int16_t signed_steering = (int16_t) ((steering >> 2) - (0xFF >> 1));

    int16_t motor_a_speed = ((int16_t) speed) + signed_steering;
    int16_t motor_b_speed = ((int16_t) speed) - signed_steering;

    if (0 > motor_a_speed)
    {
        motor_a_speed = 0;
    }
    else if (0xFFu < motor_a_speed)
    {
        motor_a_speed = 0xFFu;
    }

    if (0 > motor_b_speed)
    {
        motor_b_speed = 0;
    }
    else if (0xFFu < motor_b_speed)
    {
        motor_b_speed = 0xFFu;
    }

    l298n_motor_set_speed(MOTOR_CHANNEL_A, (uint8_t) motor_a_speed, MOTOR_DIRECTION_FORWARD);
    l298n_motor_set_speed(MOTOR_CHANNEL_B, (uint8_t) motor_b_speed, MOTOR_DIRECTION_FORWARD);
}

