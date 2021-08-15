/**
 * @file       ble_heart_rate_service.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      HRNS (BLE Heart Rate New Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_HEART_RATE_SERVICE_H
#define __BLE_HEART_RATE_SERVICE_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_HRNS_SERVICE (0x2234) /**< The UUID of the Heart Rate Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Heart Rate Service event type
 */
typedef enum
{
  BLE_HRNS_EVT_NOTIFICATION_ENABLED, /**< Heart Rate value notification enabled event. */
  BLE_HRNS_EVT_NOTIFICATION_DISABLED /**< Heart Rate value notification disabled event. */
} 
ble_hrns_evt_type_t;

/**
 * @brief Heart Rate Service event.
 */
typedef struct
{
  ble_hrns_evt_type_t evt_type;    /**< Type of event. */
  uint16_t            conn_handle; /**< Connection handle. */
}
ble_hrns_evt_t;

/* Forward declaration of the ble_hrns_t type. */
typedef struct ble_hrns_s ble_hrns_t;

/* Heart Rate Service event handler type. */
typedef void (* ble_hrns_evt_handler_t) (ble_hrns_t * p_hrns, ble_hrns_evt_t * p_evt);

/**
 * @brief Nordic Heart Rate Service initialization structure.
 */
typedef struct
{
  ble_hrns_evt_handler_t evt_handler;                    /**< Event handler to be called for handling events in the Heart Rate Service. */
  bool                   support_notification;           /**< TRUE if notification of Heart Rate measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Heart Rate characteristic */
  uint8_t                initial_heart_rate;             /**< Initial Heart Rate */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_hrns_init_t;

/**
 * @brief Nordic Heart Rate Service structure.
 */
struct ble_hrns_s
{
  uint8_t                  uuid_type;                 /**< UUID type for Heart Rate Service Base UUID. */
  ble_hrns_evt_handler_t   evt_handler;               /**< Event handler to be called for handling events in the Heart Rate Service. */
  uint16_t                 service_handle;            /**< Handle of Heart Rate Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t heart_rate_handles;        /**< Handles related to the Heart Rate characteristic. */
  uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
  uint8_t                  heart_rate_last;           /**< Last Heart Rate measurement passed to the Heart Rate Service. */
  bool                     is_notification_supported; /**< TRUE if notification of Heart Rate is supported. */
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
#define BLE_HRNS_DEF(_name)                        \
static ble_hrns_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                \
                     BLE_HRS_BLE_OBSERVER_PRIO,    \
                     ble_hrns_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                      Function for initializing the Nordic Heart Rate Service.
 *
 * @param[in]     p_hrns_init  Information needed to initialize the service.
 * 
 * @param[out]    p_hrns       Nordic Heart Rate Service structure. This structure must be supplied
 *                             by the application. It is initialized by this function and will
 *                             later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_hrns or p_hrns_init is NULL.
 */
uint32_t ble_hrns_init(ble_hrns_t *p_hrns, ble_hrns_init_t const *p_hrns_init);

/**
 * @brief                        Function for updating the Heart Rate level.
 *
 * @param[in]     p_bas          Heart Rate Service structure.
 * @param[in]     heart_rate     New Heart Rate measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_hrns_heart_rate_update(ble_hrns_t *p_hrns,
                                      uint8_t   heart_rate,
                                      uint16_t  conn_handle);

/**
 * @brief                     Function for handling the Heart Rate Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Heart Rate Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_hrns_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_HEART_RATE_SERVICE_H

/* End of file -------------------------------------------------------- */
