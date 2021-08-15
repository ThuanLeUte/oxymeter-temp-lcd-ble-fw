/**
 * @file       sys_temp.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Sytem module to handle human body temperature sensor
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sys_temp.h"
#include "bsp.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static max30205_t m_max30205;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
max30205_status_t sys_temp_init(void)
{
  m_max30205.device_address = MAX30205_I2C_ADDR;
  m_max30205.i2c_read       = bsp_i2c_read;
  m_max30205.i2c_write      = bsp_i2c_write;

  return max30205_init(&m_max30205);
}

max30205_status_t sys_temp_get(float *temp)
{
  return max30205_get_temperature(&m_max30205, temp);
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
