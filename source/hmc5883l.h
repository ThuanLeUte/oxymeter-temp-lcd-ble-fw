/**
 * @file       hmc5883l.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support HMC5883L (Magnetometer)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __HCM5883L_H
#define __HCM5883L_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "bsp.h"

/* Public defines ----------------------------------------------------- */
#define HMC5883L_I2C_ADDR                  (0x1E >> 1)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief HMC5883L sample enum
 */
typedef enum
{
   HMC5883L_SAMPLES_1 = 0x00
  ,HMC5883L_SAMPLES_2
  ,HMC5883L_SAMPLES_4
  ,HMC5883L_SAMPLES_8
}
hmc5883l_samples_t;

/**
 * @brief HMC5883L data rate enum
 */
typedef enum
{
   HMC5883L_DATARATE_0_75_HZ = 0x00
  ,HMC5883L_DATARATE_1_5HZ
  ,HMC5883L_DATARATE_3HZ
  ,HMC5883L_DATARATE_7_5HZ
  ,HMC5883L_DATARATE_15HZ
  ,HMC5883L_DATARATE_30HZ
  ,HMC5883L_DATARATE_75H
}
hmc5883l_data_rate_t;

/**
 * @brief HMC5883L range enum
 */
typedef enum
{
   HMC5883L_RANGE_0_88GA = 0x00
  ,HMC5883L_RANGE_1_3GA
  ,HMC5883L_RANGE_1_9GA
  ,HMC5883L_RANGE_2_5GA
  ,HMC5883L_RANGE_4GA
  ,HMC5883L_RANGE_4_7GA
  ,HMC5883L_RANGE_5_6GA
  ,HMC5883L_RANGE_8_1GA
}
hmc5883l_range_t;

/**
 * @brief HMC5883L mode enum
 */
typedef enum
{
   HMC5883L_CONTINOUS = 0x00
  ,HMC5883L_SINGLE
  ,HMC5883L_IDLE
}
hmc5883l_mode_t;

/**
 * @brief HMC5883L sensor struct
 */
typedef struct 
{
  uint8_t  device_address;  // I2C device address

/**
 * @brief HMC5883L data
 */
  struct
  {
    float x_axis;
    float y_axis;
    float z_axis;
  }
  data;

  // Read n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_read) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Write n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_write) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
}
hmc5883l_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize HMC5883L
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_init(hmc5883l_t *me);

/**
 * @brief         HMC5883L read raw data
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_read_raw(hmc5883l_t *me);

/**
 * @brief         HMC5883L set measurement mode
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 * @param[in]     mode          Mode
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_set_measurement_mode(hmc5883l_t *me, hmc5883l_mode_t mode);

/**
 * @brief         HMC5883L set data rate
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 * @param[in]     mode          Data rate
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_set_data_rate(hmc5883l_t *me, hmc5883l_data_rate_t data_rate);

/**
 * @brief         HMC5883L set samples
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 * @param[in]     sample        Sample
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_set_samples(hmc5883l_t *me, hmc5883l_samples_t sample);

/**
 * @brief         HMC5883L set range
 *
 * @param[in]     me            Pointer to handle of HMC5883L module.
 * @param[in]     range         Range
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t hmc5883l_set_range(hmc5883l_t *me, hmc5883l_range_t range);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __HCM5883L_H

/* End of file -------------------------------------------------------- */
