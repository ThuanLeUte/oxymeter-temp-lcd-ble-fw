/**
 * @file       l3g4200d.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support L3G4200D (Gyroscope)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "l3g4200d.h"

/* Private defines ---------------------------------------------------- */
// Registers
#define L3G4200D_REG_                   (0x00)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_l3g4200d_read_reg(l3g4200d_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);
static base_status_t m_l3g4200d_write_reg(l3g4200d_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);

/* Function definitions ----------------------------------------------- */
base_status_t l3g4200d_init(l3g4200d_t *me)
{
  uint8_t identifier = 0x00;

  if ((me == NULL) || (me->i2c_read == NULL) || (me->i2c_write == NULL))
    return BS_ERROR;

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         L3G4200D read register
 *
 * @param[in]     me      Pointer to handle of L3G4200D module.
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
static base_status_t m_l3g4200d_read_reg(l3g4200d_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_read(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}

/**
 * @brief         L3G4200D read register
 *
 * @param[in]     me      Pointer to handle of L3G4200D module.
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
static base_status_t m_l3g4200d_write_reg(l3g4200d_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_write(me->device_address, reg, p_data, len), BS_ERROR);

  return BS_OK;
}

/* End of file -------------------------------------------------------- */
