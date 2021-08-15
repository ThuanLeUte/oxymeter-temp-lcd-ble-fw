/**
 * @file       bsp_mag.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      Board support package for Magnetometer (HMC5883L)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_mag.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static hmc5883l_t m_hmc5883l;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_mag_init(void)
{
  m_hmc5883l.device_address = HMC5883L_I2C_ADDR;
  m_hmc5883l.i2c_read       = bsp_i2c_read;
  m_hmc5883l.i2c_write      = bsp_i2c_write;

  // Init
  CHECK_STATUS(hmc5883l_init(&m_hmc5883l));

  // Setting
  CHECK_STATUS(hmc5883l_set_measurement_mode(&m_hmc5883l, HMC5883L_CONTINOUS));
  CHECK_STATUS(hmc5883l_set_data_rate(&m_hmc5883l, HMC5883L_DATARATE_15HZ));
  CHECK_STATUS(hmc5883l_set_samples(&m_hmc5883l, HMC5883L_SAMPLES_1));
  CHECK_STATUS(hmc5883l_set_range(&m_hmc5883l, HMC5883L_RANGE_1_3GA));

  return BS_OK;
}

base_status_t bsp_mag_read(float *x_axis, float *y_axis, float *z_axis)
{
  CHECK_STATUS(hmc5883l_read_raw(&m_hmc5883l));

  *x_axis = m_hmc5883l.data.x_axis;
  *y_axis = m_hmc5883l.data.y_axis;
  *z_axis = m_hmc5883l.data.z_axis;

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
