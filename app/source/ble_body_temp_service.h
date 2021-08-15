/**
 * @file       ble_body_temp_service.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      BTS (BLE Body Temperature Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_BODY_TEMP_SERVICE_H
#define __BLE_BODY_TEMP_SERVICE_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_BTS_SERVICE (0x3234) /**< The UUID of the Body Temperature Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Body Temperature Service event type
 */
typedef enum
{
  BLE_BTS_EVT_NOTIFICATION_ENABLED, /**< Body Temperature value notification enabled event. */
  BLE_BTS_EVT_NOTIFICATION_DISABLED /**< Body Temperature value notification disabled event. */
} 
ble_bts_evt_type_t;

/**
 * @brief Body Temperature Service event.
 */
typedef struct
{
  ble_bts_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_bts_evt_t;

/* Forward declaration of the ble_bts_t type. */
typedef struct ble_bts_s ble_bts_t;

/* Body Temperature Service event handler type. */
typedef void (* ble_bts_evt_handler_t) (ble_bts_t * p_bts, ble_bts_evt_t * p_evt);

/**
 * @brief Nordic Body Temperature Service initialization structure.
 */
typedef struct
{
  ble_bts_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Body Temperature Service. */
  bool                   support_notification;           /**< TRUE if notification of Body Temperature measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Body Temperature characteristic */
  uint8_t                initial_body_temp;              /**< Initial Body Temperature */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_bts_init_t;

/**
 * @brief Nordic Body Temperature Service structure.
 */
struct ble_bts_s
{
  uint8_t                  uuid_type;                 /**< UUID type for Body Temperature Service Base UUID. */
  ble_bts_evt_handler_t    evt_handler;               /**< Event handler to be called for handling events in the Body Temperature Service. */
  uint16_t                 service_handle;            /**< Handle of Body Temperature Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t body_temp_handles;         /**< Handles related to the Body Temperature characteristic. */
  uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
  float                    body_temp_last;            /**< Last Body Temperature measurement passed to the Body Temperature Service. */
  bool                     is_notification_supported; /**< TRUE if notification of Body Temperature is supported. */
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
#define BLE_BTS_DEF(_name)                        \
static ble_bts_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_bts_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Body Temperature Service.
 *
 * @param[in]     p_bts_init  Information needed to initialize the service.
 * 
 * @param[out]    p_bts       Nordic Body Temperature Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_bts or p_bts_init is NULL.
 */
uint32_t ble_bts_init(ble_bts_t *p_bts, ble_bts_init_t const *p_bts_init);

/**
 * @brief                        Function for updating the Body Temperature level.
 *
 * @param[in]     p_bas          Body Temperature Service structure.
 * @param[in]     body_temp   New Body Temperature measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_bts_body_temp_update(ble_bts_t *p_bts,
                                    float     body_temp,
                                    uint16_t  conn_handle);

/**
 * @brief                     Function for handling the Nordic Body Temperature Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Body Temperature Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_bts_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_BODY_TEMP_SERVICE_H

/* End of file -------------------------------------------------------- */
