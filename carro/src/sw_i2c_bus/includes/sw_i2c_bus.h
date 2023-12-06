/*
 * sw_i2c_bus.h
 *
 *  Created on: Nov 22, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_SW_I2C_BUS_INCLUDES_SW_I2C_BUS_H_
#define SRC_SW_I2C_BUS_INCLUDES_SW_I2C_BUS_H_

#include <stdint.h>
#include <msp430.h>

void i2c_master_init(void);

void i2c_write_bytes(uint8_t i2c_slave_addr, uint8_t reg_addr, const uint8_t * const  data, uint8_t data_len);

void i2c_read_bytes(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t * out_data, uint8_t data_len);

#endif /* SRC_SW_I2C_BUS_INCLUDES_SW_I2C_BUS_H_ */
