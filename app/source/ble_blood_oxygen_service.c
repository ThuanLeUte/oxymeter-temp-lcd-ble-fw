/**
 * @file       ble_blood_oxygen_service.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      BOS (BLE Blood Oxygen Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sdk_common.h"
#include "ble.h"
#include "ble_blood_oxygen_service.h"
#include "ble_srv_common.h"
#include "nrf_log.h"

/* Private defines ---------------------------------------------------- */
#define BLE_UUID_BOS_BLOOD_OXIGEN_CHARACTERISTIC  0x1235                  /**< The UUID of the TX Characteristic. */
#define BLE_UUID_BOS_R_DATA_CHARACTERISTIC        0x1236                  /**< The UUID of the TX Characteristic. */
#define BLE_UUID_BOS_IR_DATA_CHARACTERISTIC       0x1237                  /**< The UUID of the TX Characteristic. */

#define BOS_BASE_UUID                                                                                \
  {                                                                                                  \
    {                                                                                                \
      0x41, 0xEE, 0x68, 0x3A, 0x99, 0x0F, 0x0E, 0x72, 0x85, 0x49, 0x8D, 0xB3, 0x00, 0x00, 0x00, 0x00 \
    }                                                                                                \
  } /**< Used vendor specific UUID. */

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void on_connect(ble_bos_t *p_bos, ble_evt_t const *p_ble_evt);
static void on_write(ble_bos_t *p_bos, ble_evt_t const *p_ble_evt);
static ret_code_t blood_oxygen_char_add(ble_bos_t *p_bos,
                                        const ble_bos_init_t *p_bos_init);
static ret_code_t r_data_char_add(ble_bos_t *p_bos,
                                  const ble_bos_init_t *p_bos_init);
static ret_code_t ir_data_char_add(ble_bos_t *p_bos,
                                   const ble_bos_init_t *p_bos_init);
static ret_code_t blood_oxygen_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                                 uint16_t conn_handle);
static ret_code_t r_data_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                           uint16_t conn_handle);
static ret_code_t ir_data_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                            uint16_t conn_handle);

/* Function definitions ----------------------------------------------- */
uint32_t ble_bos_init(ble_bos_t *p_bos, ble_bos_init_t const *p_bos_init)
{
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t bos_base_uuid = BOS_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_bos);
  VERIFY_PARAM_NOT_NULL(p_bos_init);

  // Initialize the service structure.
  p_bos->evt_handler               = p_bos_init->evt_handler;
  p_bos->is_notification_supported = p_bos_init->support_notification;
  p_bos->blood_oxygen_last        = 0;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&bos_base_uuid, &p_bos->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_bos->uuid_type;
  ble_uuid.uuid = BLE_UUID_BOS_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_bos->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the Characteristics.
  err_code = blood_oxygen_char_add(p_bos, p_bos_init);
  err_code = r_data_char_add(p_bos, p_bos_init);
  err_code = ir_data_char_add(p_bos, p_bos_init);

  return err_code;
}

ret_code_t ble_bos_blood_oxygen_update(ble_bos_t *p_bos,
                                       uint8_t   blood_oxygen,
                                       uint16_t  conn_handle)
{
  VERIFY_PARAM_NOT_NULL(p_bos);

  ret_code_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if (blood_oxygen != p_bos->blood_oxygen_last)
  {
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &blood_oxygen;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_bos->blood_oxygen_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
      // NRF_LOG_INFO("Blood oxygen has been updated: %d%%", blood_oxygen)

      // Save new blood oxygen value.
      p_bos->blood_oxygen_last = blood_oxygen;
    }
    else
    {
      NRF_LOG_DEBUG("Error during Blood oxygen update: 0x%08X", err_code)

      return err_code;
    }

    // Send value if connected and notifying.
    if (p_bos->is_notification_supported)
    {
      ble_gatts_hvx_params_t hvx_params;

      memset(&hvx_params, 0, sizeof(hvx_params));

      hvx_params.handle = p_bos->blood_oxygen_handles.value_handle;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.offset = gatts_value.offset;
      hvx_params.p_len  = &gatts_value.len;
      hvx_params.p_data = gatts_value.p_value;

      if (conn_handle == BLE_CONN_HANDLE_ALL)
      {
        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

        // Try sending notifications to all valid connection handles.
        for (uint32_t i = 0; i < conn_handles.len; i++)
        {
          if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          {
            if (err_code == NRF_SUCCESS)
            {
              err_code = blood_oxygen_notification_send(&hvx_params,
                                                        conn_handles.conn_handles[i]);
            }
            else
            {
              // Preserve the first non-zero error code
              UNUSED_RETURN_VALUE(blood_oxygen_notification_send(&hvx_params,
                                                            conn_handles.conn_handles[i]));
            }
          }
        }
      }
      else
      {
        err_code = blood_oxygen_notification_send(&hvx_params, conn_handle);
      }
    }
    else
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  }

  return err_code;
}

ret_code_t ble_bos_r_data_update(ble_bos_t *p_bos,
                                 uint32_t  r_data,
                                 uint16_t  conn_handle)
{
  VERIFY_PARAM_NOT_NULL(p_bos);

  ret_code_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if (r_data != p_bos->r_data_last)
  {
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint32_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *)&r_data;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_bos->r_data_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
      // NRF_LOG_INFO("Blood oxygen has been updated: %d%%", blood_oxygen)

      // Save new blood oxygen value.
      p_bos->r_data_last = r_data;
    }
    else
    {
      NRF_LOG_DEBUG("Error during Blood oxygen update: 0x%08X", err_code)

      return err_code;
    }

    // Send value if connected and notifying.
    if (p_bos->is_notification_supported)
    {
      ble_gatts_hvx_params_t hvx_params;

      memset(&hvx_params, 0, sizeof(hvx_params));

      hvx_params.handle = p_bos->r_data_handles.value_handle;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.offset = gatts_value.offset;
      hvx_params.p_len  = &gatts_value.len;
      hvx_params.p_data = gatts_value.p_value;

      if (conn_handle == BLE_CONN_HANDLE_ALL)
      {
        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

        // Try sending notifications to all valid connection handles.
        for (uint32_t i = 0; i < conn_handles.len; i++)
        {
          if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          {
            if (err_code == NRF_SUCCESS)
            {
              err_code = r_data_notification_send(&hvx_params,
                                                  conn_handles.conn_handles[i]);
            }
            else
            {
              // Preserve the first non-zero error code
              UNUSED_RETURN_VALUE(r_data_notification_send(&hvx_params,
                                                            conn_handles.conn_handles[i]));
            }
          }
        }
      }
      else
      {
        err_code = r_data_notification_send(&hvx_params, conn_handle);
      }
    }
    else
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  }

  return err_code;
}

ret_code_t ble_bos_ir_data_update(ble_bos_t *p_bos,
                                  uint32_t  ir_data,
                                  uint16_t  conn_handle)
{
  VERIFY_PARAM_NOT_NULL(p_bos);

  ret_code_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if (ir_data != p_bos->ir_data_last)
  {
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint32_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *)&ir_data;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_bos->ir_data_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
      // NRF_LOG_INFO("Blood oxygen has been updated: %d%%", blood_oxygen)

      // Save new blood oxygen value.
      p_bos->ir_data_last = ir_data;
    }
    else
    {
      NRF_LOG_DEBUG("Error during Blood oxygen update: 0x%08X", err_code)

      return err_code;
    }

    // Send value if connected and notifying.
    if (p_bos->is_notification_supported)
    {
      ble_gatts_hvx_params_t hvx_params;

      memset(&hvx_params, 0, sizeof(hvx_params));

      hvx_params.handle = p_bos->ir_data_handles.value_handle;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.offset = gatts_value.offset;
      hvx_params.p_len  = &gatts_value.len;
      hvx_params.p_data = gatts_value.p_value;

      if (conn_handle == BLE_CONN_HANDLE_ALL)
      {
        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

        // Try sending notifications to all valid connection handles.
        for (uint32_t i = 0; i < conn_handles.len; i++)
        {
          if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          {
            if (err_code == NRF_SUCCESS)
            {
              err_code = ir_data_notification_send(&hvx_params,
                                                   conn_handles.conn_handles[i]);
            }
            else
            {
              // Preserve the first non-zero error code
              UNUSED_RETURN_VALUE(ir_data_notification_send(&hvx_params,
                                                            conn_handles.conn_handles[i]));
            }
          }
        }
      }
      else
      {
        err_code = ir_data_notification_send(&hvx_params, conn_handle);
      }
    }
    else
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  }

  return err_code;
}

void ble_bos_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
  {
    return;
  }

  ble_bos_t *p_bos = (ble_bos_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_bos, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    on_write(p_bos, p_ble_evt);
    break;

  default:
    break;
  }
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for adding the Blood Oxygen characteristic.
 *
 * @param[in]     p_bos         Blood Oxygen Service structure.
 * @param[in]     p_bos_init    Information needed to initialize the service.
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t blood_oxygen_char_add(ble_bos_t *p_bos, const ble_bos_init_t *p_bos_init)
{
  ret_code_t              err_code;
  ble_add_char_params_t   add_char_params;
  ble_add_descr_params_t  add_descr_params;
  uint8_t                 initial_blood_oxygen;
  uint8_t                 init_len;
  uint8_t                 encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

  // Add blood oxygen characteristic
  initial_blood_oxygen = p_bos_init->initial_blood_oxygen;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_BOS_BLOOD_OXIGEN_CHARACTERISTIC;
  add_char_params.max_len           = sizeof(uint8_t);
  add_char_params.init_len          = sizeof(uint8_t);
  add_char_params.p_init_value      = &initial_blood_oxygen;
  add_char_params.char_props.notify = p_bos->is_notification_supported;
  add_char_params.char_props.read   = 1;
  add_char_params.cccd_write_access = p_bos_init->bl_cccd_wr_sec;
  add_char_params.read_access       = p_bos_init->bl_rd_sec;

  err_code = characteristic_add(p_bos->service_handle,
                                &add_char_params,
                                &(p_bos->blood_oxygen_handles));
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  if (p_bos_init->p_report_ref != NULL)
  {
    // Add Report Reference descriptor
    init_len = ble_srv_report_ref_encode(encoded_report_ref, p_bos_init->p_report_ref);

    memset(&add_descr_params, 0, sizeof(add_descr_params));
    add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
    add_descr_params.read_access = p_bos_init->bl_report_rd_sec;
    add_descr_params.init_len    = init_len;
    add_descr_params.max_len     = add_descr_params.init_len;
    add_descr_params.p_value     = encoded_report_ref;

    err_code = descriptor_add(p_bos->blood_oxygen_handles.value_handle,
                              &add_descr_params,
                              &p_bos->report_ref_handle);
    return err_code;
  }
  else
  {
    p_bos->report_ref_handle = BLE_GATT_HANDLE_INVALID;
  }

  return NRF_SUCCESS;
}

/**
 * @brief         Function for adding the r data characteristic.
 *
 * @param[in]     p_bos         Blood Oxygen Service structure.
 * @param[in]     p_bos_init    Information needed to initialize the service.
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t r_data_char_add(ble_bos_t *p_bos, const ble_bos_init_t *p_bos_init)
{
  ret_code_t              err_code;
  ble_add_char_params_t   add_char_params;
  ble_add_descr_params_t  add_descr_params;
  uint32_t                initial_r_data;
  uint8_t                 init_len;
  uint8_t                 encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

  // Add blood oxygen characteristic
  initial_r_data = p_bos_init->initial_r_data;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_BOS_R_DATA_CHARACTERISTIC;
  add_char_params.max_len           = sizeof(uint32_t);
  add_char_params.init_len          = sizeof(uint32_t);
  add_char_params.p_init_value      = (uint8_t *)&initial_r_data;
  add_char_params.char_props.notify = p_bos->is_notification_supported;
  add_char_params.char_props.read   = 1;
  add_char_params.cccd_write_access = p_bos_init->bl_cccd_wr_sec;
  add_char_params.read_access       = p_bos_init->bl_rd_sec;

  err_code = characteristic_add(p_bos->service_handle,
                                &add_char_params,
                                &(p_bos->r_data_handles));
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  if (p_bos_init->p_report_ref != NULL)
  {
    // Add Report Reference descriptor
    init_len = ble_srv_report_ref_encode(encoded_report_ref, p_bos_init->p_report_ref);

    memset(&add_descr_params, 0, sizeof(add_descr_params));
    add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
    add_descr_params.read_access = p_bos_init->bl_report_rd_sec;
    add_descr_params.init_len    = init_len;
    add_descr_params.max_len     = add_descr_params.init_len;
    add_descr_params.p_value     = encoded_report_ref;

    err_code = descriptor_add(p_bos->r_data_handles.value_handle,
                              &add_descr_params,
                              &p_bos->report_ref_handle);
    return err_code;
  }
  else
  {
    p_bos->report_ref_handle = BLE_GATT_HANDLE_INVALID;
  }

  return NRF_SUCCESS;
}

/**
 * @brief         Function for adding the Ir data characteristic.
 *
 * @param[in]     p_bos         Blood Oxygen Service structure.
 * @param[in]     p_bos_init    Information needed to initialize the service.
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t ir_data_char_add(ble_bos_t *p_bos, const ble_bos_init_t *p_bos_init)
{
  ret_code_t              err_code;
  ble_add_char_params_t   add_char_params;
  ble_add_descr_params_t  add_descr_params;
  uint32_t                initial_ir_data;
  uint8_t                 init_len;
  uint8_t                 encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

  // Add blood oxygen characteristic
  initial_ir_data = p_bos_init->initial_ir_data;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_BOS_IR_DATA_CHARACTERISTIC;
  add_char_params.max_len           = sizeof(uint32_t);
  add_char_params.init_len          = sizeof(uint32_t);
  add_char_params.p_init_value      = (uint8_t *)&initial_ir_data;
  add_char_params.char_props.notify = p_bos->is_notification_supported;
  add_char_params.char_props.read   = 1;
  add_char_params.cccd_write_access = p_bos_init->bl_cccd_wr_sec;
  add_char_params.read_access       = p_bos_init->bl_rd_sec;

  err_code = characteristic_add(p_bos->service_handle,
                                &add_char_params,
                                &(p_bos->ir_data_handles));
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  if (p_bos_init->p_report_ref != NULL)
  {
    // Add Report Reference descriptor
    init_len = ble_srv_report_ref_encode(encoded_report_ref, p_bos_init->p_report_ref);

    memset(&add_descr_params, 0, sizeof(add_descr_params));
    add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
    add_descr_params.read_access = p_bos_init->bl_report_rd_sec;
    add_descr_params.init_len    = init_len;
    add_descr_params.max_len     = add_descr_params.init_len;
    add_descr_params.p_value     = encoded_report_ref;

    err_code = descriptor_add(p_bos->ir_data_handles.value_handle,
                              &add_descr_params,
                              &p_bos->report_ref_handle);
    return err_code;
  }
  else
  {
    p_bos->report_ref_handle = BLE_GATT_HANDLE_INVALID;
  }

  return NRF_SUCCESS;
}

/**
 * @brief         Function for sending notifications with the r data characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t r_data_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                                 uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
  if (err_code == NRF_SUCCESS)
  {
    // NRF_LOG_INFO("r data notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                  err_code,
                  conn_handle);
  }
  return err_code;
}

/**
 * @brief         Function for sending notifications with the ir data characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t ir_data_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                                 uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
  if (err_code == NRF_SUCCESS)
  {
    // NRF_LOG_INFO("ir data notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                  err_code,
                  conn_handle);
  }
  return err_code;
}

/**
 * @brief         Function for sending notifications with the Blood oxygen Level characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t blood_oxygen_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                                 uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
  if (err_code == NRF_SUCCESS)
  {
    NRF_LOG_INFO("Blood oxygen notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                  err_code,
                  conn_handle);
  }
  return err_code;
}

/**
 * @brief         Function for handling the Connect event.
 *
 * @param[in]     p_bos       Blood Oxygen Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_connect(ble_bos_t *p_bos, ble_evt_t const *p_ble_evt)
{

}

/**
 * @brief         Function for handling the Write event.
 *
 * @param[in]     p_bos       Blood Oxygen Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_write(ble_bos_t *p_bos, ble_evt_t const *p_ble_evt)
{

}

/* End of file -------------------------------------------------------- */
