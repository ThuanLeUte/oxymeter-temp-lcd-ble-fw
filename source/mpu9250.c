/**
 * @file       mpu9250.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support MPU9250 (IMU)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "mpu9250.h"

/* Private defines ---------------------------------------------------- */
#define MPU9250_REG_WHO_AM_I          (0x75)
#define MPU9250_REG_PWR_MAGT_1        (0x6B)
#define MPU9250_REG_CONFIG            (0x1A)
#define MPU9250_REG_GYRO_CONFIG       (0x1B)
#define MPU9250_REG_ACCEL_CONFIG      (0x1C)
#define MPU9250_REG_SMPLRT_DIV        (0x19)
#define MPU9250_REG_INT_STATUS        (0x3A)
#define MPU9250_REG_ACCEL_XOUT_H      (0x3B)
#define MPU9250_REG_TEMP_OUT_H        (0x41)
#define MPU9250_REG_GYRO_XOUT_H       (0x43)
#define MPU9250_REG_FIFO_EN           (0x23)
#define MPU9250_REG_INT_ENABLE        (0x38)
#define MPU9250_REG_I2CMACO           (0x23)
#define MPU9250_REG_USER_CNT          (0x6A)
#define MPU9250_REG_FIFO_COUNTH       (0x72)
#define MPU9250_REG_FIFO_R_W          (0x74)

#define MPU9250_PART_IDENTIFIER       (0X68)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_mpu9250_read_reg(mpu9250_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);
static base_status_t m_mpu9250_write_reg(mpu9250_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);

/* Function definitions ----------------------------------------------- */
base_status_t mpu9250_init(mpu9250_t *me)
{
  uint8_t identifier;

  if ((me == NULL) || (me->i2c_read == NULL) || (me->i2c_write == NULL))
    return BS_ERROR_PARAMS;

  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_WHO_AM_I, &identifier, 1));

  CHECK((MPU9250_PART_IDENTIFIER == identifier), BS_ERROR);

  CHECK_STATUS(mpu9250_config(me));

  return BS_OK;
}

base_status_t mpu9250_config(mpu9250_t *me)
{
  uint8_t value;

  // Reset device
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_PWR_MAGT_1, 0x80, 1));

  // Delay for device reset
  me->delay(100);

  // Clock Source
  value = me->config.clock_source & 0x07;            // Change the 7th bits of register
  value |= me->config.sleep_mode_bit << 6) & 0x40;   // Change only the 7th bit in the register
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_PWR_MAGT_1, value, 1));

  // Should wait 10ms after changing the clock setting
  me->delay(10);

  // Set the digital low pass filter
  value = me->config.config_dlpf & 0x07;
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_CONFIG, value, 1));

  // Select the gyroscope full scale range
  value = (me->config.gyro_full_scale << 3) & 0x18;
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_GYRO_CONFIG, value, 1));

  // Select the accelerometer full scale range
  value = (me->config.accel_full_scale << 3) & 0x18;
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_ACCEL_CONFIG, value, 1));

  // Set sample rate devider to default
  CHECK_STATUS(mpu9250_get_sample_rate_divider(me, 0x04));

  // Accelerometer scaling factor
  switch (me->configconfig.accel_full_scale)
  {
  case MPU9250_ACCEL_FS_SEL_2G : me->accel.scale_factor = (2000.0f / 32768.0f);  break;
  case MPU9250_ACCEL_FS_SEL_4G : me->accel.scale_factor = (4000.0f / 32768.0f);  break;
  case MPU9250_ACCEL_FS_SEL_8G : me->accel.scale_factor = (8000.0f / 32768.0f);  break;
  case MPU9250_ACCEL_FS_SEL_16G: me->accel.scale_factor = (16000.0f / 32768.0f); break;
  default : break;
  }

  // Gyroscope scaling factor
  switch (me.config->gyro_full_scale)
  {
  case MPU9250_GYRO_FS_SEL_250 : me->gyro.scale_factor = 250.0f / 32768.0f;  break;
  case MPU9250_GYRO_FS_SEL_500 : me->gyro.scale_factor = 500.0f / 32768.0f;  break;
  case MPU9250_GYRO_FS_SEL_1000: me->gyro.scale_factor = 1000.0f / 32768.0f; break;
  case MPU9250_GYRO_FS_SEL_2000: me->gyro.scale_factor = 2000.0f / 32768.0f; break;
  default : break;
  }

  return BS_OK;
}

base_status_t mpu9250_get_sample_rate_divider(mpu9250_t *me, uint8_t *value)
{
  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_SMPLRT_DIV, value, 1));

  return BS_OK;
}

base_status_t mpu9250_set_sample_rate_divider(mpu9250_t *me, uint8_t value)
{
  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_SMPLRT_DIV, &value, 1));

  return BS_OK;
}

base_status_t mpu9250_get_external_frame_sync(mpu9250_t *me, uint8_t *value)
{
  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_CONFIG, value, 1));

  *value &= 0x38;
  *value >>= 3;

  return BS_OK;
}

base_status_t mpu9250_set_external_frame_sync(mpu9250_t *me, enum mpu9250_ext_frame_t ext_frame)
{
  uint8_t value;

  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_CONFIG, &value, 1));

  value &= ~0x38;
  value |= (ext_frame << 3);

  CHECK_STATUS(m_mpu9250_write_reg(me, MPU9250_REG_CONFIG,  value, 1));

  return BS_OK;
}

base_status_t mpu9250_get_accel_raw_data(mpu9250_t *me)
{
  uint8_t status;
  uint8_t buffer[6]

  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_INT_STATUS, &status, 1));

  if ((status && 0x01))
  {
    CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_ACCEL_XOUT_H, buffer, 6));

    me->accel.raw_data.x = ((buffer[0] << 8) + buffer[1]);
    me->accel.raw_data.y = ((buffer[2] << 8) + buffer[3]);
    me->accel.raw_data.z = ((buffer[4] << 8) + buffer[5]);
  }
  else
  {
    return BS_ERROR;
  }

  return BS_OK;
}

base_status_t mpu9250_get_accel_scale_data(mpu9250_t *me)
{
  CHECK_STATUS(mpu9250_get_accel_raw_data(me));

  me->accel.scaled_data.x = ((me->accel.raw_data.x + 0.0f) * me->accel.scale_factor);
  me->accel.scaled_data.y = ((me->accel.raw_data.y + 0.0f) * me->accel.scale_factor);
  me->accel.scaled_data.z = ((me->accel.raw_data.z + 0.0f) * me->accel.scale_factor);

  return BS_OK;
}

base_status_t mpu9250_get_accel_calib_data(mpu9250_t *me)
{
  CHECK_STATUS(mpu9250_get_accel_scale_data(me));

  me->accel.calib_data.x = me->accel.scaled_data.x - me->bias.x;
  me->accel.calib_data.y = me->accel.scaled_data.y - me->bias.y;
  me->accel.calib_data.z = me->accel.scaled_data.z - me->bias.z;

  return BS_OK;
}

base_status_t mpu9250_get_gyro_raw_data(mpu9250_t *me)
{
  uint8_t status;
  uint8_t buffer[6]

  CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_INT_STATUS, &status, 1));

  if ((status && 0x01))
  {
    CHECK_STATUS(m_mpu9250_read_reg(me, MPU9250_REG_GYRO_XOUT_H, buffer, 6));

    me->gyro.raw_data.x = ((buffer[0] << 8) + buffer[1]);
    me->gyro.raw_data.y = ((buffer[2] << 8) + buffer[3]);
    me->gyro.raw_data.z = ((buffer[4] << 8) + buffer[5]);
  }
  else
  {
    return BS_ERROR;
  }

  return BS_OK;
}

base_status_t mpu9250_get_gyro_scale_data(mpu9250_t *me)
{
  CHECK_STATUS(mpu9250_get_gyro_raw_data(me));

  me->gyro.scaled_data.x = me->gyro.raw_data.x * me->gyro.scale_factor;
  me->gyro.scaled_data.y = me->gyro.raw_data.y * me->gyro.scale_factor;
  me->gyro.scaled_data.z = me->gyro.raw_data.z * me->gyro.scale_factor;

  return BS_OK;
}

base_status_t mpu9250_accel_calib(mpu9250_t *me, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
  me->bias.x = (x_max + x_min) / 2.0f;
  me->bias.y = (y_max + y_min) / 2.0f;
  me->bias.z = (z_max + z_min) / 2.0f;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         MPU9250 read register
 *
 * @param[in]     me      Pointer to handle of MPU9250 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_mpu9250_read_reg(mpu9250_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_read(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}

/**
 * @brief         MPU9250 read register
 *
 * @param[in]     me      Pointer to handle of MPU9250 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_mpu9250_write_reg(mpu9250_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_write(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}


/* End of file -------------------------------------------------------- */
