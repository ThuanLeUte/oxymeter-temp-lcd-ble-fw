/**
 * @file       bsp_accelero.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      Board support package for Accelerometer (MIS2DH)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_accelero.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static mis2dh_t m_mis2dh;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_accel_init(void)
{
  m_mis2dh.device_address = MIS2DH_I2C_ADDR;
  m_mis2dh.i2c_read       = bsp_i2c_read;
  m_mis2dh.i2c_write      = bsp_i2c_write;

  mis2dh_init(&m_mis2dh);
}

base_status_t bsp_accel_get_raw_data(mis2dh_raw_data_t *raw_data)
{
  CHECK_STATUS(mis2dh_get_raw_data(&m_mis2dh));

  raw_data = &m_mis2dh.raw_data;

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
