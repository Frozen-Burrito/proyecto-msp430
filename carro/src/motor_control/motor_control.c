/*
 * motor_control.c
 *
 *  Created on: Dec 1, 2023
 *      Author: Fernando Mendoza
 */
#include "motor_control.h"

#ifndef MOTOR_FORWARD_ONLY
#define REVERSE_SPEED_BIT   ((uint8_t) 0x80u)
#endif

void motor_control_init(void)
{
    l298n_init(MOTOR_CHANNEL_A | MOTOR_CHANNEL_B);
}

void motor_control(uint8_t steering, uint8_t speed)
{
    int8_t signed_steering = (int8_t) (steering - (0xFF >> 1));

    uint8_t direction = MOTOR_DIRECTION_FORWARD;

#ifndef MOTOR_FORWARD_ONLY
    if (REVERSE_SPEED_BIT & speed)
    {
        direction = MOTOR_DIRECTION_REVERSE;
        speed &= ~REVERSE_SPEED_BIT;
    }
#endif

    int16_t motor_a_speed = ((int16_t) speed);

    if (0u < speed)
    {
        motor_a_speed += signed_steering;
    }

    int16_t motor_b_speed = ((int16_t) speed);

    if (0u < speed)
    {
        motor_b_speed -= signed_steering;
    }

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

    l298n_motor_set_speed(MOTOR_CHANNEL_A, (uint8_t) motor_a_speed, direction);
    l298n_motor_set_speed(MOTOR_CHANNEL_B, (uint8_t) motor_b_speed, direction);
}

