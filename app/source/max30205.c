/**
 * @file       max30205.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      Driver support MAX30205 (Human Body Temperature Sensor)
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "max30205.h"
#include "nrf_log.h"

/* Private defines ---------------------------------------------------- */
#define MAX30205_TEMPERATURE      (0x00)
#define MAX30205_CONFIGURATION    (0x01)
#define MAX30205_THYST            (0x02)
#define MAX30205_TOS              (0x03)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define CHECK(expr, ret)            \
  do {                              \
    if (!(expr)) {                  \
      NRF_LOG_INFO("%s", #expr);    \
      return (ret);                 \
    }                               \
  } while (0)

#define MAX_CHECK(expr)             \
  do {                              \
    max30205_status_t ret = (expr); \
    if (MAX30205_OK != ret) {       \
      NRF_LOG_INFO("%s", #expr);    \
      return (ret);                 \
    }                               \
  } while (0)

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static max30205_status_t m_max30205_read_reg(max30205_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);
static max30205_status_t m_max30205_write_reg(max30205_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);

/* Function definitions ----------------------------------------------- */
max30205_status_t max30205_init(max30205_t *me)
{
  uint8_t buffer = 0x00;

  if ((me == NULL) || (me->i2c_read == NULL) || (me->i2c_write == NULL))
    return MAX30205_ERR_PARAM;

  me->device_address = MAX30205_I2C_ADDR;

  MAX_CHECK(m_max30205_write_reg(me, MAX30205_CONFIGURATION, &buffer, 1));
  MAX_CHECK(m_max30205_write_reg(me, MAX30205_THYST, &buffer, 1));
  MAX_CHECK(m_max30205_write_reg(me, MAX30205_TOS, &buffer, 1));

  return MAX30205_OK;
}

max30205_status_t max30205_get_temperature(max30205_t *me, float *temperature)
{
  uint8_t buffer[2] = {0};

  MAX_CHECK(m_max30205_read_reg(me, MAX30205_TEMPERATURE, buffer, 2)); // Read two bytes

  int16_t raw_temp = buffer[0] << 8 | buffer[1];                       // Combine two bytes

  *temperature = raw_temp * 0.00390625;                                // Convert to temperature

  return MAX30205_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         MAX30205 read register
 *
 * @param[in]     me      Pointer to handle of MAX30205 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_I2C
 */
static max30205_status_t m_max30205_read_reg(max30205_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_read(me->device_address, reg, p_data, len), MAX30205_ERR_I2C);

  return MAX30205_OK;
}

/**
 * @brief         MAX30205 read register
 *
 * @param[in]     me      Pointer to handle of MAX30205 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_I2C
 */
static max30205_status_t m_max30205_write_reg(max30205_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_write(me->device_address, reg, p_data, len), MAX30205_ERR_I2C);

  return MAX30205_OK;
}

/* Undefine macros ---------------------------------------------------- */
#undef CHECK
#undef MAX_CHECK
/* End of file -------------------------------------------------------- */
