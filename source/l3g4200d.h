/**
 * @file       l3g4200d.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support L3G4200D (Gyroscope)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __L3G4200D_H
#define __L3G4200D_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "bsp.h"

/* Public defines ----------------------------------------------------- */
#define L3G4200D_I2C_ADDR                  (0x90 >> 1)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief L3G4200D sensor struct
 */
typedef struct 
{
  uint8_t  device_address;  // I2C device address

  // Read n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_read) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Write n-bytes from device's internal address <reg_addr> via I2C bus
  base_status_t (*i2c_write) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
}
l3g4200d_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize L3G4200D
 *
 * @param[in]     me            Pointer to handle of L3G4200D module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t l3g4200d_init(l3g4200d_t *me);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __L3G4200D_H

/* End of file -------------------------------------------------------- */
