/*
 * sw_i2c_bus.c
 *
 *  Created on: Nov 22, 2023
 *      Author: Fernando Mendoza
 */
#include "sw_i2c_bus.h"
#include "sw_i2c_bus_config.h"

#define READ_BIT (BIT0)

#define LOW    (0u)
#define HIGH   (1u)

static void scl(uint8_t dato);
static void sda(uint8_t dato);

static void sw_i2c_start(void);
static void sw_i2c_stop(void);

static unsigned char sw_i2c_ack_input(void);
static void sw_i2c_ack_output(uint8_t nack);

static void sw_i2c_send_byte(uint8_t data);
static uint8_t sw_i2c_receive_byte();

void i2c_master_init(void)
{
    P1SEL &= ~SW_I2C_SDA_PIN;
    P1SEL2 &= ~SW_I2C_SDA_PIN;
    P1OUT |= SW_I2C_SDA_PIN;
    P1DIR |= SW_I2C_SDA_PIN;

    P2SEL &= ~SW_I2C_SCL_PIN;
    P2SEL2 &= ~SW_I2C_SCL_PIN;
    P2OUT |= SW_I2C_SCL_PIN;
    P2DIR |= SW_I2C_SCL_PIN;
}

void i2c_write_bytes(uint8_t i2c_slave_addr, uint8_t reg_addr, const uint8_t * const  data, uint8_t data_len)
{
    if (0u == data_len || 0u == data) return;

    uint8_t rw_slave_addr = (i2c_slave_addr << 1u) & ~READ_BIT;
    uint8_t tx_byte_index = 0u;

    sw_i2c_start();

    sw_i2c_send_byte(rw_slave_addr);

    if (LOW != sw_i2c_ack_input()) return;

    sw_i2c_send_byte(reg_addr);

    if (LOW != sw_i2c_ack_input()) return;

    do
    {
        sw_i2c_send_byte(data[tx_byte_index]);

        if (LOW != sw_i2c_ack_input()) return;

        tx_byte_index++;

    } while (tx_byte_index < data_len);


    sw_i2c_stop();
}

void i2c_read_bytes(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t * out_data, uint8_t data_len)
{
    if (0u == data_len || 0u == out_data) return;

    uint8_t rw_slave_addr = (i2c_slave_addr << 1u) & ~READ_BIT;
    uint8_t rx_byte_index = 0u;

    sw_i2c_start();

    sw_i2c_send_byte(rw_slave_addr);

    if (LOW != sw_i2c_ack_input()) return;

    sw_i2c_send_byte(reg_addr);

    if (LOW != sw_i2c_ack_input()) return;

    sw_i2c_start();

    rw_slave_addr |= READ_BIT;
    sw_i2c_send_byte(rw_slave_addr);

    if (LOW != sw_i2c_ack_input()) return;

    do
    {
        out_data[rx_byte_index] = sw_i2c_receive_byte();

        rx_byte_index++;

        sw_i2c_ack_output((rx_byte_index == data_len));

    } while (rx_byte_index < data_len);

    sw_i2c_stop();
}

void sw_i2c_start(void)
{
    scl(LOW);
    sda(HIGH);
    scl(HIGH);

    // Start hold time (>= 600 ns).
    __delay_cycles(1);

    sda(LOW);

    // Start setup time (>= 600 ns).
    __delay_cycles(1);

    scl(LOW);
}

void sw_i2c_stop(void)
{
    scl(LOW);
    sda(LOW);
    scl(HIGH);

    // Start hold time (>= 600 ns).
    __delay_cycles(1);

    sda(HIGH);

    // Start setup time (>= 600 ns).
    __delay_cycles(1);

    scl(LOW);
    scl(HIGH);
}

void sw_i2c_send_byte(uint8_t data)
{
    uint8_t count = 8u;

    do
    {
        sda(0u != (data & BIT7));
        scl(HIGH);

        // Clock pulse width high (>= 600 ns).
        __delay_cycles(1);

        scl(LOW);

        // Clock pulse width low (>= 1300 ns).
        __delay_cycles(1);
        data <<= 1u;

    } while (--count);

    sda(LOW);
}

uint8_t sw_i2c_receive_byte()
{
    uint8_t count = 8u;
    uint8_t rx_byte;

    P1DIR &= ~SW_I2C_SDA_PIN;

    do
    {
        scl(HIGH);
        rx_byte <<= 1;
        if (0u != (P1IN & SW_I2C_SDA_PIN)) rx_byte++;
        scl(LOW);
        __delay_cycles(1);

    } while (--count);

    P1DIR |= SW_I2C_SDA_PIN;

    return rx_byte;
}

uint8_t sw_i2c_ack_input(void)
{
    uint8_t ack;

    P1DIR &= ~SW_I2C_SDA_PIN;
    scl(HIGH);

    ack = 0u != (P1IN & SW_I2C_SDA_PIN);

    scl(LOW);
    P1DIR |= SW_I2C_SDA_PIN;

    return ack;
}

void sw_i2c_ack_output(uint8_t nack)
{
    sda(nack);
    scl(HIGH);

    __delay_cycles(1);

    scl(LOW);
    __delay_cycles(1);
    sda(HIGH);
}

void scl(uint8_t output_level)
{
    if (HIGH == output_level) P2OUT |= SW_I2C_SCL_PIN;
    else P2OUT &= ~SW_I2C_SCL_PIN;
}

void sda(uint8_t output_level)
{
    if (HIGH == output_level) P1OUT |= SW_I2C_SDA_PIN;
    else P1OUT &= ~SW_I2C_SDA_PIN;
}
