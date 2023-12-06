/*
 * mpu6050_config.h
 *
 *  Created on: Nov 20, 2023
 *      Author: f3rm3
 */

#ifndef SRC_MPU6050_CFG_MPU6050_CONFIG_H_
#define SRC_MPU6050_CFG_MPU6050_CONFIG_H_
#include <stdint.h>
#include "gpio.h"

#define MPU6050_I2C_ADDR    ((uint8_t) 0x68u)
//#define MPU6050_I2C_ADDR    ((uint8_t) 0x69)

#define MPU6050_INTR_PIN    ((uint16_t) 3u)
#define MPU6050_INTR_PORT   (GPIO_PORT1)

#define MPU6050_SAMPLE_RATE_DIVIDER ((uint8_t) 80u)
#define MPU6050_SAMPLE_PERIOD_MS (100u)

//#define MPU6050_HARDWARE_I2C

#ifndef MPU6050_CALIBRATION_MODE
//#define MPU6050_CALIBRATION_MODE
#endif

#define MPU6050_ACCE_X_OFFSET       ((int16_t) 1619)
#define MPU6050_ACCE_Y_OFFSET       ((int16_t) 1987)
#define MPU6050_ACCE_Z_OFFSET       ((int16_t) 886)
#define MPU6050_GYRO_X_OFFSET       ((int16_t) 0xFFE5)
#define MPU6050_GYRO_Y_OFFSET       ((int16_t) 0x015F)
#define MPU6050_GYRO_Z_OFFSET       ((int16_t) 0x0014)

#endif /* SRC_MPU6050_CFG_MPU6050_CONFIG_H_ */
