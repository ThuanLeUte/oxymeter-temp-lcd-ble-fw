/**
 * @file       mpu9250.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support MPU9250 (IMU)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "bsp.h"

/* Public defines ----------------------------------------------------- */
#define MPU9250_I2C_ADDR                  (0x90 >> 1)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief MPU9250 clock source enum
 */
typedef enum
{
   MPU9250_CLOCK_INTERNAL_20MHZ   = 0x00
  ,MPU9250_CLOCK_PLL              = 0x01
  ,MPU9250_CLOCK_PLL              = 0x02
  ,MPU9250_CLOCK_PLL              = 0x03
  ,MPU9250_CLOCK_PLL              = 0x04
  ,MPU9250_CLOCK_PLL              = 0x04
  ,MPU9250_CLOCK_INTERNAL_20MHZ   = 0x00
  ,MPU9250_CLOCK_TIM_GEN_IN_RESET = 0x07
}
mpu9250_clock_source_t;

/**
 * @brief MPU9250 gyro full scale range enum
 */
typedef enum
{
   MPU9250_GYRO_FS_SEL_250  = 0x00
  ,MPU9250_GYRO_FS_SEL_500  = 0x01
  ,MPU9250_GYRO_FS_SEL_1000 = 0x02
  ,MPU9250_GYRO_FS_SEL_2000 = 0x03
}
mpu9250_gyro_full_scale_t;

/**
 * @brief MPU9250 accel full scale range enum
 */
typedef enum
{
   MPU9250_ACCEL_FS_SEL_2G   = 0x00
  ,MPU9250_ACCEL_FS_SEL_4G   = 0x01
  ,MPU9250_ACCEL_FS_SEL_8G   = 0x02
  ,MPU9250_ACCEL_FS_SEL_16G  = 0x03
}
mpu9250_accel_full_scale_t;

/**
 * @brief MPU9250 digital low pass filter enum
 */
typedef enum
{
   MPU9250_DIGI_LPF_260A_256G_HZ = 0x00
  ,MPU9250_DIGI_LPF_184A_188G_HZ = 0x01
  ,MPU9250_DIGI_LPF_94A_98G_HZ   = 0x02
  ,MPU9250_DIGI_LPF_44A_42G_HZ   = 0x03
  ,MPU9250_DIGI_LPF_21A_20G_HZ   = 0x04
  ,MPU9250_DIGI_LPF_10_HZ        = 0x05
  ,MPU9250_DIGI_LPF_5_HZ         = 0x06
}
mpu9250_digi_low_pass_filter_t;

/**
 * @brief MPU9250 external frame sysnchronization enum
 */
typedef enum
{
   MPU9250_INPUT_DISABLE = 0x00
  ,MPU9250_TEMP_OUT_L    = 0x01
  ,MPU9250_GYRO_XOUT_L   = 0x02
  ,MPU9250_GYRO_YOUT_L   = 0x03
  ,MPU9250_GYRO_ZOUT_L   = 0x04
  ,MPU9250_ACCEL_XOUT_L  = 0x05
  ,MPU9250_ACCEL_YOUT_L  = 0x06
  ,MPU9250_ACCEL_ZOUT_L  = 0x07
}
mpu9250_ext_frame_t;

/**
 * @brief MPU9250 configuration enum
 */
typedef struct
{
  mpu9250_clock_source_t clock_source;
  mpu9250_gyro_full_scale_t gyro_full_scale;
  mpu9250_accel_full_scale_t accel_full_scale;
  mpu9250_digi_low_pass_filter_t digi_low_pass_filter;
  bool sleep_mode_bit;
}
mpu9250_config_t;

/**
 * @brief MPU9250 raw data structure
 */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}
mpu9250_raw_data_t;

/**
 * @brief MPU9250 scaled data structure
 */
typedef struct
{
  float x;
  float y;
  float z;
}
mpu9250_scaled_data_t;

/**
 * @brief MPU9250 sensor struct
 */
typedef struct 
{
  uint8_t  device_address;  // I2C device address
  mpu9250_config_t config;
  mpu9250_scaled_data_t scaled_data;
  mpu9250_raw_data_t raw_data;
  mpu9250_scaled_data_t bias;

/**
 * @brief Accel structure
 */
  struct
  {
    mpu9250_scaled_data_t scaled_data;
    mpu9250_scaled_data_t calib_data;
    mpu9250_raw_data_t raw_data;
    float scale_factor;
  }
  accel;

/**
 * @brief Gyro structure
 */
  struct
  {
    mpu9250_scaled_data_t scaled_data;
    mpu9250_raw_data_t raw_data;
    float scale_factor;
  }
  gyro;

  // Read n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_read) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Write n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_write) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Delay
  void (*delay) (uint16_t ms);
}
mpu9250_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize MPU9250
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_init(mpu9250_t *me);

/**
 * @brief         MPU9250 initialization configuration
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_config(mpu9250_t *me);

/**
 * @brief         MPU9250 get sample rate divider
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 * @param[in]     value         Value config
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_sample_rate_divider(mpu9250_t *me,  uint8_t value);

/**
 * @brief         MPU9250 set sample rate divider
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 * @param[in]     value         Value
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_set_sample_rate_divider(mpu9250_t *me, uint8_t value);

/**
 * @brief         MPU9250 get external frame sync
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 * @param[in]     value         Value
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_external_frame_sync(mpu9250_t *me, uint8_t *value);

/**
 * @brief         MPU9250 set external frame sync
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 * @param[in]     ext_frame     Value of external frame
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_set_external_frame_sync(mpu9250_t *me, mpu9250_ext_frame_t ext_frame);

/**
 * @brief         MPU9250 get accel raw data
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_accel_raw_data(mpu9250_t *me);

/**
 * @brief         MPU9250 get accel scaled data (g unit of gravity, 1g = 9.81m/s2)
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_accel_scale_data(mpu9250_t *me);

/**
 * @brief         MPU9250 get accel calibrated data
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_accel_calib_data(mpu9250_t *me);

/**
 * @brief         MPU9250 get gyro raw data
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_gyro_raw_data(mpu9250_t *me);

/**
 * @brief         MPU9250 get gyro scaled data
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_get_gyro_scale_data(mpu9250_t *me);

/**
 * @brief         MPU9250 accel calib
 *
 * @param[in]     me            Pointer to handle of MPU9250 module.
 * @param[in]     x_min         Variable x_min
 * @param[in]     x_max         Variable x_max
 * @param[in]     y_min         Variable y_min
 * @param[in]     y_max         Variable y_max
 * @param[in]     z_min         Variable z_min
 * @param[in]     z_max         Variable z_max
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t mpu9250_accel_calib(mpu9250_t *me, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __MPU9250_H

/* End of file -------------------------------------------------------- */
