/**
 * @file       sys_po.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      Sytem module to handle Pulse Oximeter (PO)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SYS_PO_H
#define __SYS_PO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "ob1203.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         System Pulse oximeter run
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void sys_po_main(void);

/**
 * @brief         System Pulse oximeter heart rate ble notify
 *
 * @param[in]     heart_rate    Heart Rate
 *
 * @attention     None
 *
 * @return        None
 */
void sys_po_heart_rate_notify(uint8_t heart_rate);

/**
 * @brief         System Pulse oximeter blood oxigen ble notify
 *
 * @param[in]     blood_oxygen    Blood oxygen
 *
 * @attention     None
 *
 * @return        None
 */
void sys_po_blood_oxygen_notify(uint8_t blood_oxygen);

/**
 * @brief         System Pulse oximeter r data ble notify
 *
 * @param[in]     r_data            r data
 *
 * @attention     None
 *
 * @return        None
 */
void sys_po_r_data_notify(uint32_t r_data);

/**
 * @brief         System Pulse oximeter ir data ble notify
 *
 * @param[in]     ir_data           ir data
 *
 * @attention     None
 *
 * @return        None
 */
void sys_po_ir_data_notify(uint32_t ir_data);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __SYS_PO_H

/* End of file -------------------------------------------------------- */
