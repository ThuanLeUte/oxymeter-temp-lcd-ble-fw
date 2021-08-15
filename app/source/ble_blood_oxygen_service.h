/**
 * @file       ble_blood_oxygen_service.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      BOS (BLE Blood Oxygen Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_BLOOD_OXYGEN_SERVICE_H
#define __BLE_BLOOD_OXYGEN_SERVICE_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_BOS_SERVICE (0x1234) /**< The UUID of the Blood Oxygen Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Blood Oxygen Service event type
 */
typedef enum
{
  BLE_BOS_EVT_NOTIFICATION_ENABLED, /**< Blood Oxygen value notification enabled event. */
  BLE_BOS_EVT_NOTIFICATION_DISABLED /**< Blood Oxygen value notification disabled event. */
} 
ble_bos_evt_type_t;

/**
 * @brief Blood Oxygen Service event.
 */
typedef struct
{
  ble_bos_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_bos_evt_t;

/* Forward declaration of the ble_bos_t type. */
typedef struct ble_bos_s ble_bos_t;

/* Blood Oxygen Service event handler type. */
typedef void (* ble_bos_evt_handler_t) (ble_bos_t * p_bos, ble_bos_evt_t * p_evt);

/**
 * @brief Nordic Blood Oxygen Service initialization structure.
 */
typedef struct
{
  ble_bos_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Blood oxygen Service. */
  bool                   support_notification;           /**< TRUE if notification of Blood oxygen measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Blood oxygen characteristic */
  uint8_t                initial_blood_oxygen;           /**< Initial Blood oxygen */
  uint8_t                initial_r_data;                 /**< Initial R data */
  uint8_t                initial_ir_data;                /**< Initial IR data */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_bos_init_t;

/**
 * @brief Nordic Blood Oxygen Service structure.
 */
struct ble_bos_s
{
  uint8_t                  uuid_type;                 /**< UUID type for Blood Oxygen Service Base UUID. */
  ble_bos_evt_handler_t    evt_handler;               /**< Event handler to be called for handling events in the Blood oxygen Service. */
  uint16_t                 service_handle;            /**< Handle of Blood oxygen Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t blood_oxygen_handles;      /**< Handles related to the Blood oxygen characteristic. */
  ble_gatts_char_handles_t r_data_handles;            /**< Handles related to the R data characteristic. */
  ble_gatts_char_handles_t ir_data_handles;           /**< Handles related to the IR data characteristic. */
  uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
  uint8_t                  blood_oxygen_last;         /**< Last Blood oxygen measurement passed to the Blood oxygen Service. */
  uint32_t                 r_data_last;               /**< Last r data measurement passed to the Blood oxygen Service. */
  uint32_t                 ir_data_last;              /**< Last ir data measurement passed to the Blood oxygen Service. */
  bool                     is_notification_supported; /**< TRUE if notification of Blood oxygen is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_nus instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_BOS_DEF(_name)                        \
static ble_bos_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_bos_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Blood Oxygen Service.
 *
 * @param[in]     p_bos_init  Information needed to initialize the service.
 * 
 * @param[out]    p_bos       Nordic Blood Oxygen Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_bos or p_bos_init is NULL.
 */
uint32_t ble_bos_init(ble_bos_t *p_bos, ble_bos_init_t const *p_bos_init);

/**
 * @brief                        Function for updating the blood oxygen level.
 *
 * @param[in]     p_bas          Blood oxygen Service structure.
 * @param[in]     blood_oxygen   New blood oxygen measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_bos_blood_oxygen_update(ble_bos_t *p_bos,
                                       uint8_t   blood_oxygen,
                                       uint16_t  conn_handle);

/**
 * @brief                        Function for updating the r data.
 *
 * @param[in]     p_bas          Blood oxygen Service structure.
 * @param[in]     r_data         New r data measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_bos_r_data_update(ble_bos_t *p_bos,
                                 uint32_t   r_data,
                                 uint16_t   conn_handle);

/**
 * @brief                        Function for updating ir data.
 *
 * @param[in]     p_bas          Blood oxygen Service structure.
 * @param[in]     ir_data        New ir data measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_bos_ir_data_update(ble_bos_t *p_bos,
                                  uint32_t   ir_data,
                                  uint16_t  conn_handle);
/**
 * @brief                     Function for handling the Nordic Blood Oxygen Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Blood Oxygen Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_bos_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_BLOOD_OXYGEN_SERVICE_H

/* End of file -------------------------------------------------------- */
