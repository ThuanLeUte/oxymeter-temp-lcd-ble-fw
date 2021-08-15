/**
 * @file       hmc5883l.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support HMC5883L (Magnetometer)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "hmc5883l.h"

/* Private defines ---------------------------------------------------- */
#define HMC5883L_REG_CONFIG_A              (0X00)
#define HMC5883L_REG_CONFIG_B              (0X01)
#define HMC5883L_REG_MODE                  (0X02)
#define HMC5883L_REG_OUTPUT_X_MSB          (0X03)
#define HMC5883L_REG_OUTPUT_X_LSB          (0X04)
#define HMC5883L_REG_OUTPUT_Y_MSB          (0X05)
#define HMC5883L_REG_OUTPUT_Y_LSB          (0X06)
#define HMC5883L_REG_OUTPUT_Z_MSB          (0X07)
#define HMC5883L_REG_OUTPUT_Z_LSB          (0X08)
#define HMC5883L_REG_STATUS                (0X09)
#define HMC5883L_REG_IDENT_A               (0X10)
#define HMC5883L_REG_IDENT_B               (0X20)
#define HMC5883L_REG_IDENT_C               (0X30)

#define HMC5883L_VALUE_IDENT_A             (0X48)
#define HMC5883L_VALUE_IDENT_B             (0X34)
#define HMC5883L_VALUE_IDENT_C             (0X33)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_hmc5883l_read_reg(hmc5883l_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);
static base_status_t m_hmc5883l_write_reg(hmc5883l_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);

/* Function definitions ----------------------------------------------- */
base_status_t hmc5883l_init(hmc5883l_t *me)
{
  uint8_t identifier[3];

  if ((me == NULL) || (me->i2c_read == NULL) || (me->i2c_write == NULL))
    return BS_ERROR_PARAMS;

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_IDENT_A, &identifier[0], 1));
  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_IDENT_B, &identifier[1], 1));
  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_IDENT_C, &identifier[2], 1));

  CHECK(HMC5883L_VALUE_IDENT_A == identifier[0]);
  CHECK(HMC5883L_VALUE_IDENT_B == identifier[1]);
  CHECK(HMC5883L_VALUE_IDENT_C == identifier[2]);

  return BS_OK;
}

base_status_t hmc5883l_read_raw(hmc5883l_t *me)
{
  uint8_t buffer[2];

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_OUTPUT_X_MSB, buffer, 2));
  me->data.x_axis = buffer[0] << 8 | buffer[1];

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_OUTPUT_Y_MSB, buffer, 2));
  me->data.y_axis = buffer[0] << 8 | buffer[1];

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_OUTPUT_Z_MSB, buffer, 2));
  me->data.z_axis = buffer[0] << 8 | buffer[1];

  return BS_OK;
}

base_status_t hmc5883l_set_measurement_mode(hmc5883l_t *me, hmc5883l_mode_t mode)
{
  uint8_t value;

  CHECK_STATUS(hmc5883l_read_reg(me, HMC5883L_REG_MODE, &value, 1));

  value &= 0b11111100;
  value |= mode;

  CHECK_STATUS(hmc5883l_write_reg(me, HMC5883L_REG_MODE, value, 1));

  return BS_OK;
}

base_status_t hmc5883l_set_data_rate(hmc5883l_t *me, hmc5883l_data_rate_t data_rate)
{
  uint8_t value;

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_CONFIG_A, &value, 1));

  value &= 0b11100011;
  value |= (sample << 2);

  CHECK_STATUS(m_hmc5883l_write_reg(me, HMC5883L_REG_CONFIG_A, value, 1));

  return BS_OK;
}

base_status_t hmc5883l_set_samples(hmc5883l_t *me, hmc5883l_samples_t sample)
{
  uint8_t value;

  CHECK_STATUS(m_hmc5883l_read_reg(me, HMC5883L_REG_CONFIG_A, &value, 1));

  value &= 0b10011111;
  value |= (sample << 5);

  CHECK_STATUS(m_hmc5883l_write_reg(me, HMC5883L_REG_CONFIG_A, value, 1));

  return BS_OK;
}

base_status_t hmc5883l_set_range(hmc5883l_t *me, hmc5883l_range_t range)
{
  CHECK_STATUS(m_hmc5883l_write_reg(me, HMC5883L_REG_CONFIG_B, range << 5, 1));

  return BS_OK;
}

/**
 * @brief         HMC5883L read register
 *
 * @param[in]     me      Pointer to handle of HMC5883L module.
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
static base_status_t m_hmc5883l_read_reg(hmc5883l_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_read(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}

/**
 * @brief         HMC5883L write register
 *
 * @param[in]     me      Pointer to handle of HMC5883L module.
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
static base_status_t m_hmc5883l_write_reg(hmc5883l_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_write(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}

/* End of file -------------------------------------------------------- */
