/*
 * mpu6050.c
 *
 *  Created on: Nov 20, 2023
 *      Author: Fernando Mendoza
 */
#include <stdint.h>
#include "mpu6050.h"
#include "mpu6050_config.h"

#define MPU6050_RA_XA_OFFS_H        (0x06u)
#define MPU6050_RA_XG_OFFS_USRH     (0x13u)
#define MPU6050_RA_SMPLRT_DIV       (0x19u)
#define MPU6050_RA_GYRO_CONFIG      (0x1Bu)
#define MPU6050_RA_ACCEL_CONFIG     (0x1Cu)
#define MPU6050_RA_INTR_PIN_CFG     (0x37u)
#define MPU6050_RA_INTR_ENABLE      (0x38u)
#define MPU6050_RA_INTR_STATUS      (0x3Au)
#define MPU6050_RA_ACCEL_XOUT_H     (0x3Bu)
#define MPU6050_RA_ACCEL_ZOUT_H     (0x3Fu)
#define MPU6050_RA_GYRO_XOUT_H      (0x43u)
#define MPU6050_RA_TEMP_XOUT_H      (0x41u)
#define MPU6050_RA_PWR_MGMT_1       (0x6Bu)
#define MPU6050_RA_WHO_AM_I         (0x75u)

#define MPU6050_FULL_READ_LEN       (14u)

#define MPU6050_PWR_MGMT_SLEEP_BIT  (BIT6)

#define MPU6050_GYRO_FS_250 (0x00u)
#define MPU6050_ACCE_FS_2G  (0x00u)

#define MPU6050_INTR_LEVEL_LOW      (BIT7)
#define MPU6050_INTR_OPEN           (BIT6)
#define MPU6050_INTR_LATCH_EN       (BIT5)
#define MPU6050_INTR_ANY_RD_CLEAR   (BIT4)

#define MPU6050_DATA_READY_FLAG     (BIT0)

#define MPU6050_GPIO_INTR_FLAG      ((1u << MPU6050_INTR_PIN) << (8u * (MPU6050_INTR_PORT - 1u)))

#define RAD_TO_DEG              (_IQ(57.296))
#define GYRO_FACTOR             (_IQ(0.98))
#define ACCE_FACTOR             (_IQ(0.02))

#define DT                      (_IQdiv(_IQ(MPU6050_SAMPLE_PERIOD_MS), _IQ(1000)))
#define GYRO_TO_ANGLE           (_IQdiv(DT, _IQ(131)))

static void mpu6050_load_offsets(void);

static uint8_t mpu6050_read_interrupt_flags(void);

static uint8_t mpu6050_rw_buf[MPU6050_FULL_READ_LEN];
static uint8_t mpu6050_acce_offsets[6u] = {};
static uint8_t mpu6050_gyro_offsets[6u] = {};

uint8_t mpu6050_init(void)
{
    uint8_t mpu6050_i2c_addr = 0x00u;
    uint8_t mpu6050_pwr_mgmt_1;

    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_WHO_AM_I, (uint8_t *) &mpu6050_i2c_addr, sizeof mpu6050_i2c_addr);

    if (MPU6050_I2C_ADDR != mpu6050_i2c_addr)
    {
        return 1u;
    }

    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, (uint8_t *) &mpu6050_pwr_mgmt_1, 1u);

    if (MPU6050_PWR_MGMT_SLEEP_BIT & mpu6050_pwr_mgmt_1)
    {
        mpu6050_pwr_mgmt_1 &= ~MPU6050_PWR_MGMT_SLEEP_BIT;
        i2c_write_bytes(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, (uint8_t *) &mpu6050_pwr_mgmt_1, 1u);

        i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, (uint8_t *) &mpu6050_pwr_mgmt_1, 1u);
    }

    if (0u != (MPU6050_PWR_MGMT_SLEEP_BIT & mpu6050_pwr_mgmt_1))
    {
        return 1u;
    }

    mpu6050_rw_buf[0u] = MPU6050_SAMPLE_RATE_DIVIDER;
    i2c_write_bytes(MPU6050_I2C_ADDR, MPU6050_RA_SMPLRT_DIV, mpu6050_rw_buf, 1u);

    mpu6050_load_offsets();

    return 0u;
}

void mpu6050_interrupt_en(void)
{
    mpu6050_rw_buf[0u] = (MPU6050_INTR_LEVEL_LOW | MPU6050_INTR_LATCH_EN | MPU6050_INTR_ANY_RD_CLEAR);
    mpu6050_rw_buf[1u] = MPU6050_DATA_READY_FLAG;

    i2c_write_bytes(MPU6050_I2C_ADDR, MPU6050_RA_INTR_PIN_CFG, mpu6050_rw_buf, 2u);

    gpio_alloc_isr(MPU6050_INTR_PORT, MPU6050_INTR_PIN, GPIO_ISR_H_TO_L_FLAG);
}

void mpu6050_accel_pitch_roll(int16_t * const out_accel_x, int16_t * const out_pitch, int16_t * const out_roll)
{
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;

    volatile _iq angle_x;
    volatile _iq angle_y;

    static _iq prev_angle_x = _IQ(0);
    static _iq prev_angle_y = _IQ(0);

    if (mpu6050_has_data_ready())
    {
        mpu6050_read_acce(&raw_accel_x, &raw_accel_y, &raw_accel_z);
        mpu6050_read_gyro(&raw_gyro_x, &raw_gyro_y, 0u);

        if (0 != raw_accel_z)
        {
            int32_t accel_x = ((int32_t) raw_accel_x);
            int32_t accel_y = ((int32_t) raw_accel_y);
            int32_t squared_accel_z = (((int32_t) raw_accel_z) * ((int32_t) raw_accel_z));

            volatile _iq acce_angle_x = _IQmpy(_IQatan(_IQdiv(_IQ(accel_x), _IQ3toIQ(_IQ3sqrt(_IQ3((accel_y * accel_y) + squared_accel_z))))), RAD_TO_DEG);
            volatile _iq acce_angle_y = _IQmpy(_IQatan(_IQdiv(_IQ(accel_y), _IQ3toIQ(_IQ3sqrt(_IQ3((accel_x * accel_x) + squared_accel_z))))), RAD_TO_DEG);

            angle_x = _IQmpy(GYRO_FACTOR, (_IQmpy(_IQ(raw_gyro_x), GYRO_TO_ANGLE) + prev_angle_x)) + _IQmpy(ACCE_FACTOR, acce_angle_x);
            prev_angle_x = angle_x;

            angle_y = _IQmpy(GYRO_FACTOR, (_IQmpy(_IQ(raw_gyro_y), GYRO_TO_ANGLE) + prev_angle_y)) + _IQmpy(ACCE_FACTOR, acce_angle_y);
            prev_angle_y = angle_y;

            *out_accel_x = raw_accel_x;
            *out_pitch = _IQint(angle_x);
            *out_roll = _IQint(angle_y);
        }
    }
}

uint8_t mpu6050_read_acce_x(int16_t * const out_acce_x)
{
    if (0u == out_acce_x) return 1u;

    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, mpu6050_rw_buf, 2u);

    *out_acce_x = (int16_t) ((((uint16_t) mpu6050_rw_buf[0]) << 8) | mpu6050_rw_buf[1]);

    return 0u;
}

uint8_t mpu6050_read_acce_z(int16_t * const out_acce_z)
{
    if (0u == out_acce_z) return 1u;

    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_ZOUT_H, mpu6050_rw_buf, 2u);

    *out_acce_z = (int16_t) ((((uint16_t) mpu6050_rw_buf[0]) << 8) | mpu6050_rw_buf[1]);

    return 0u;
}


uint8_t mpu6050_read_acce(int16_t * const out_acce_x, int16_t * const out_acce_y, int16_t * const out_acce_z)
{
    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, mpu6050_rw_buf, 6u);

    if (0u != out_acce_x)
    {
        *out_acce_x = (int16_t) ((((uint16_t) mpu6050_rw_buf[0]) << 8) | mpu6050_rw_buf[1]);
    }

    if (0u != out_acce_y)
    {
        *out_acce_y = (int16_t) ((((uint16_t) mpu6050_rw_buf[2]) << 8) | mpu6050_rw_buf[3]);
    }

    if (0u != out_acce_z)
    {
        *out_acce_z = (int16_t) ((((uint16_t) mpu6050_rw_buf[4]) << 8) | mpu6050_rw_buf[5]);
    }

    return 0u;
}

uint8_t mpu6050_read_gyro(int16_t * const out_gyro_x, int16_t * const out_gyro_y, int16_t * const out_gyro_z)
{
    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_XOUT_H, mpu6050_rw_buf, 6u);

    if (0u != out_gyro_x)
    {
        *out_gyro_x = (int16_t) ((((uint16_t) mpu6050_rw_buf[0]) << 8) | mpu6050_rw_buf[1]);
    }

    if (0u != out_gyro_y)
    {
        *out_gyro_y = (int16_t) ((((uint16_t) mpu6050_rw_buf[2]) << 8) | mpu6050_rw_buf[3]);
    }

    if (0u != out_gyro_z)
    {
        *out_gyro_z = (int16_t) ((((uint16_t) mpu6050_rw_buf[4]) << 8) | mpu6050_rw_buf[5]);
    }

    return 0u;
}

uint8_t mpu6050_has_data_ready(void)
{
    if (MPU6050_GPIO_INTR_FLAG & isr_flags)
    {
        uint8_t mpu6050_interrupt_status = mpu6050_read_interrupt_flags();

        return (MPU6050_DATA_READY_FLAG & mpu6050_interrupt_status);
    }
    else
    {
        return 0u;
    }
}

uint8_t mpu6050_read_interrupt_flags(void)
{
    uint8_t interrupt_flags = 0x00u;

    i2c_read_bytes(MPU6050_I2C_ADDR, MPU6050_RA_INTR_STATUS, &interrupt_flags, 1u);

    return interrupt_flags;
}

void mpu6050_load_offsets(void)
{
    mpu6050_acce_offsets[0u] = MPU6050_ACCE_X_OFFSET >> 8u;
    mpu6050_acce_offsets[1u] = MPU6050_ACCE_X_OFFSET & 0xFFu;
    mpu6050_acce_offsets[2u] = MPU6050_ACCE_Y_OFFSET >> 8u;
    mpu6050_acce_offsets[3u] = MPU6050_ACCE_Y_OFFSET & 0xFFu;
    mpu6050_acce_offsets[4u] = MPU6050_ACCE_Z_OFFSET >> 8u;
    mpu6050_acce_offsets[5u] = MPU6050_ACCE_Z_OFFSET & 0xFFu;

    i2c_write_bytes(MPU6050_I2C_ADDR, MPU6050_RA_XA_OFFS_H, mpu6050_acce_offsets, 6u);
}
