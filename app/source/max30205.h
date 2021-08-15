/**
 * @file       max30205.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      Driver support MAX30205 (Human Body Temperature Sensor)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __MAX30205_H
#define __MAX30205_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
#define MAX30205_I2C_ADDR                (0x90 >> 1)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief MAX30205 status
 */
typedef enum
{
  MAX30205_OK = 0x00,
  MAX30205_ERR_PARAM,
  MAX30205_ERR_I2C
}
max30205_status_t;

/**
 * @brief MAX30205 sensor struct
 */
typedef struct 
{
  uint8_t  device_address;  // I2C device address

  // Read n-bytes from device's internal address <reg_addr> via I2C bus
  int (*i2c_read) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Write n-bytes from device's internal address <reg_addr> via I2C bus
  int (*i2c_write) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
}
max30205_t;


/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize MAX30205
 *
 * @param[in]     me            Pointer to handle of MAX30205 module.
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_PARAM
 * - MAX30205_ERR_I2C 
 */
max30205_status_t max30205_init(max30205_t *me);

/**
 * @brief         Initialize MAX30205
 *
 * @param[in]     me            Pointer to handle of MAX30205 module.
 * @param[in]     temperature   Pointer to temperature.
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_I2C 
 */
max30205_status_t max30205_get_temperature(max30205_t *me, float *temperature);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __MAX30205_H

/* End of file -------------------------------------------------------- */
