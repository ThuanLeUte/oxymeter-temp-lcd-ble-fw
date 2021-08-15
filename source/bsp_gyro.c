/**
 * @file       bsp_gyro.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      Board support package for Gyroscope (L3G4200D)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_gyro.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static l3g4200d_t m_l3g4200d;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_gyro_init(void)
{
  m_l3g4200d.device_address = L3G4200D_I2C_ADDR;
  m_l3g4200d.i2c_read       = bsp_i2c_read;
  m_l3g4200d.i2c_write      = bsp_i2c_write;

  l3g4200d_init(&m_l3g4200d);
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
