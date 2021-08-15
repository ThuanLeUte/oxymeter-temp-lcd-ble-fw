/**
 * @file       gc9a01.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support GC9A01 (LCD driver)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "gc9a01.h"
#include "bsp_io_11.h"

/* Private defines ---------------------------------------------------- */
#define GC9A01_CMD_MODE           (0)
#define GC9A01_DATA_MODE          (1)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_gc9a01_run_cfg_script(gc9a01_t *me);

/* Function definitions ----------------------------------------------- */
base_status_t gc9a01_init(gc9a01_t *me)
{
  if ((me == NULL) || (me->spi_send == NULL))
    return BS_ERROR_PARAMS;

  m_gc9a01_run_cfg_script(me);

  return BS_OK;
}

base_status_t gc9a01_display_control(gc9a01_t *me, bool enable)
{
  if (enable)
    gc9a01_write_cmd(me, GC9A01_SLPOUT);
  else
    gc9a01_write_cmd(me, GC9A01_SLPIN);

  return BS_OK;
}

base_status_t gc9a01_write_cmd(gc9a01_t *me, uint8_t cmd)
{
  me->gpio_write(IO_LCD_DC, GC9A01_CMD_MODE);
  CHECK(BS_OK == me->spi_send(&cmd, 1), BS_ERROR);

  return BS_OK;
}

base_status_t gc9a01_write_data(gc9a01_t *me, uint8_t *data, uint16_t len)
{
  me->gpio_write(IO_LCD_DC, GC9A01_DATA_MODE);
  CHECK(BS_OK == me->spi_send(data, len), BS_ERROR);

  return BS_OK;
}

base_status_t gc9a01_write_data_byte(gc9a01_t *me, uint8_t data)
{
  me->gpio_write(IO_LCD_DC, GC9A01_DATA_MODE);
  CHECK(BS_OK == me->spi_send(&data, 1), BS_ERROR);

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         GC9A01 run cfg script
 *
 * @param[in]     me      Pointer to handle of GC9A01 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_gc9a01_run_cfg_script(gc9a01_t *me)
{
  int i          = 0;
  int end_script = 0;

  do
  {
    switch (GC9A01_CFG_SCRIPT[i].cmd)
    {
    case GC9A01_START:
      break;
    case GC9A01_CMD:
      gc9a01_write_cmd(me, GC9A01_CFG_SCRIPT[i].data & 0xFF);
      break;
    case GC9A01_DATA:
      gc9a01_write_data_byte(me, GC9A01_CFG_SCRIPT[i].data & 0xFF);
      break;
    case GC9A01_DELAY:
      me->delay_ms(GC9A01_CFG_SCRIPT[i].data);
      break;
    case GC9A01_END:
      end_script = 1;
      break;
    }
    i++;
  }
  while (!end_script);

  return BS_OK;
}

/* End of file -------------------------------------------------------- */
