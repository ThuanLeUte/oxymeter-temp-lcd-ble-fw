/**
 * @file       bsp_imu.C
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      Board support package for IMU (MPU9250)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_imu.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static mpu9250_t m_mpu9250;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_imu_init(void)
{
  // Init
  m_mpu9250.device_address = MPU9250_I2C_ADDR;
  m_mpu9250.i2c_read       = bsp_i2c_read;
  m_mpu9250.i2c_write      = bsp_i2c_write;

  // Configaration
  m_mpu9250.config.clock_source         = MPU9250_CLOCK_INTERNAL_20MHZ;
  m_mpu9250.config.gyro_full_scale      = MPU9250_GYRO_FS_SEL_500;
  m_mpu9250.config.accel_full_scale     = MPU9250_ACCEL_FS_SEL_4G;
  m_mpu9250.config.digi_low_pass_filter = MPU9250_DIGI_LPF_184A_188G_HZ;
  m_mpu9250.config.sleep_mode_bit       = 0;  // 1: Sleep mode, 0: Normal mode

  return mpu9250_init(&m_mpu9250);
}

base_status_t bsp_gyro_accel_get(mpu9250_scaled_data_t *accel_scaled, mpu9250_scaled_data_t *gyro_scaled)
{
  mpu9250_get_accel_scale_data(&m_mpu9250);
  mpu9250_get_gyro_scale_data(&m_mpu9250);

  memcpy(accel_scaled, &m_mpu9250.accel.scaled_data, sizeof(m_mpu9250.accel.scaled_data));
  memcpy(gyro_scaled, &m_mpu9250.gyro.scaled_data, sizeof(m_mpu9250.gyro.scaled_data));
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
