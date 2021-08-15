/**
 * @file       sys_bm.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      System module to handle Battery Monitor (BM)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_BATT_H
#define _SYS_BATT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Battery monitor init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void sys_bm_init(void);

/**
 * @brief         Battery monitor get value
 *
 * @param[in]     p_vbat       Pointer to battery voltage in mV
 *
 * @attention     None
 *
 * @return        None
 */
void sys_bm_get(uint16_t * p_vbatt);

/**
 * @brief         Battery monitor get level in percent
 *
 * @param[in]     batt_level    Pointer to battery level in percent
 *
 * @attention     None
 *
 * @return        None
 */
void sys_bm_get_level_in_percent(uint8_t *batt_level);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // _SYS_BATT_H

/* End of file -------------------------------------------------------- */
