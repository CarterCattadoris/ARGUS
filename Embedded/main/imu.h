#ifndef IMU_H
#define IMU_H

#include "esp_err.h"

/* MPU6050 I2C address (AD0 pin low) */
#define MPU6050_ADDR        0x68

/* I2C GPIO assignments — change if these conflict */
#define IMU_SDA_GPIO        2
#define IMU_SCL_GPIO        1
#define IMU_I2C_FREQ_HZ     400000   /* 400 kHz fast-mode */

/* MPU6050 register map (subset) */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CFG     0x1B
#define MPU6050_REG_ACCEL_CFG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

/* Length of the accel + temp + gyro burst read */
#define MPU6050_RAW_DATA_LEN    14

/* UDP telemetry destination */
#define IMU_TARGET_IP       "10.144.113.22"  /* Pi 5 IP — update as needed */
#define IMU_TARGET_PORT     9877

/* Task parameters */
#define IMU_TASK_STACK      4096
#define IMU_TASK_PRIORITY   5
#define IMU_SEND_INTERVAL_MS 20   /* ~50 Hz */

/**
 * Initialise the I2C bus and MPU6050, then start the IMU
 * streaming task.  Call once from app_main().
 */
esp_err_t imu_init(void);

#endif /* IMU_H */