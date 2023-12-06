/*
 * mpu6050.h
 *
 *  Created on: Nov 20, 2023
 *      Author: f3rm3
 */

#ifndef SRC_MPU6050_INCLUDE_MPU6050_H_
#define SRC_MPU6050_INCLUDE_MPU6050_H_

#include <stdint.h>

#ifndef GLOBAL_IQ
#define GLOBAL_IQ    15
#endif

#include <IQmathLib.h>

#include "gpio.h"
#include "mpu6050_config.h"

#ifdef MPU6050_HARDWARE_I2C
#include "i2c_bus.h"
#else
#include "sw_i2c_bus.h"
#endif

uint8_t mpu6050_init(void);

void mpu6050_interrupt_en(void);

void mpu6050_accel_pitch_roll(int16_t * const out_accel_x, int16_t * const out_pitch, int16_t * const out_roll);

uint8_t mpu6050_read_acce_x(int16_t * const out_acce_x);

uint8_t mpu6050_read_acce_z(int16_t * const out_acce_z);

uint8_t mpu6050_read_acce(int16_t * const out_acce_x, int16_t * const out_acce_y, int16_t * const out_acce_z);

uint8_t mpu6050_read_gyro(int16_t * const out_gyro_x, int16_t * const out_gyro_y, int16_t * const out_gyro_z);

uint8_t mpu6050_has_data_ready(void);

#endif /* SRC_MPU6050_INCLUDE_MPU6050_H_ */
