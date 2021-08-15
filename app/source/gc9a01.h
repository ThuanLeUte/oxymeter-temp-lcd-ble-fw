/**
 * @file       gc9a01.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-22
 * @author     Thuan Le
 * @brief      Driver support GC9A01 (LCD driver)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __GC9A01_H
#define __GC9A01_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "gc9a01_defs.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief GC9A01 sensor struct
 */
typedef struct 
{
  uint8_t  device_address;  // I2C device address

  // Send one byte via SPI bus
  base_status_t (*spi_send) (uint8_t *tx_data, uint16_t len);

  // Delay ms
  void (*delay_ms) (uint32_t ms);

  // Gpio set
  void (*gpio_write) (uint8_t pin, uint8_t state);
}
gc9a01_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize GC9A01
 *
 * @param[in]     me     Pointer to handle of GC9A01 module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t gc9a01_init(gc9a01_t *me);

/**
 * @brief         Display contro
 *
 * @param[in]     me      Pointer to handle of GC9A01 module.
 * @param[in]     enable  Enable display
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t gc9a01_display_control(gc9a01_t *me, bool enable);

/**
 * @brief         GC9A01 write comamnd
 *
 * @param[in]     me      Pointer to handle of GC9A01 module.
 * @param[in]     cmd     Comamnd
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t gc9a01_write_cmd(gc9a01_t *me, uint8_t cmd);

/**
 * @brief         GC9A01 write 1 byte data
 *
 * @param[in]     me      Pointer to handle of GC9A01 module.
 * @param[in]     data    Data to write
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t gc9a01_write_data_byte(gc9a01_t *me, uint8_t data);

/**
 * @brief         GC9A01 write data
 *
 * @param[in]     me      Pointer to handle of GC9A01 module.
 * @param[in]     data    Pointer to data
 * @param[in]     len     Data lenght
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t gc9a01_write_data(gc9a01_t *me, uint8_t *data, uint16_t len);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __GC9A01_H

/* End of file -------------------------------------------------------- */
