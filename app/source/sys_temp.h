/**
 * @file       sys_temp.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Sytem module to handle human body temperature sensor
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SYS_TEMP_H
#define __SYS_TEMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "max30205.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         System temperature sensor init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_PARAM
 * - MAX30205_ERR_I2C
 */
max30205_status_t sys_temp_init(void);

/**
 * @brief         System temperature sensor get
 *
 * @param[in]     temp      Pointer to handler temperature data
 *
 * @attention     None
 *
 * @return
 * - MAX30205_OK
 * - MAX30205_ERR_I2C
 */
max30205_status_t sys_temp_get(float *temp);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __SYS_TEMP_H

/* End of file -------------------------------------------------------- */
