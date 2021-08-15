/**
 * @file       sys_bm.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      System module to handle Battery Monitor (BM)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "nrf_log.h"
#include "sys_bm.h"
#include "nrf_drv_saadc.h"

/* Private defines ---------------------------------------------------- */
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/6 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.

#define BATT_VOLTAGE_MIN      660
#define BATT_VOLTAGE_MAX      1100

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

#define BATTERY_CHANNEL_CONFIG(PIN_P)             \
{                                                 \
  .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
  .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
  .gain       = NRF_SAADC_GAIN1_6,                \
  .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
  .acq_time   = NRF_SAADC_ACQTIME_10US,           \
  .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
  .burst      = NRF_SAADC_BURST_DISABLED,         \
  .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
  .pin_n      = NRF_SAADC_INPUT_DISABLED          \
}

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static nrf_saadc_value_t m_adc_buf;                   //!< Buffer used for storing ADC value.
static uint16_t          m_batt_lvl_in_milli_volts;   //!< Current battery level.

/* Private function prototypes ---------------------------------------- */
static void m_saadc_event_handler(nrf_drv_saadc_evt_t const * p_evt);

/* Function definitions ----------------------------------------------- */
void sys_bm_init(void)
{
  nrf_saadc_channel_config_t config =
      BATTERY_CHANNEL_CONFIG(NRF_SAADC_INPUT_AIN2);

  ret_code_t err_code = nrf_drv_saadc_init(NULL, m_saadc_event_handler);

  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_channel_init(0, &config);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_buffer_convert(&m_adc_buf, 1);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_sample();
  APP_ERROR_CHECK(err_code);
}

void sys_bm_get(uint16_t * p_vbatt)
{
  VERIFY_PARAM_NOT_NULL_VOID(p_vbatt);

  *p_vbatt = m_batt_lvl_in_milli_volts;
  if (!nrf_drv_saadc_is_busy())
  {
    ret_code_t err_code = nrf_drv_saadc_buffer_convert(&m_adc_buf, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
  }
}

void sys_bm_get_level_in_percent(uint8_t *batt_level)
{
  VERIFY_PARAM_NOT_NULL_VOID(batt_level);

  if (BATT_VOLTAGE_MAX <= m_batt_lvl_in_milli_volts)
    *batt_level = 100;
  else if (BATT_VOLTAGE_MIN >= m_batt_lvl_in_milli_volts)
    *batt_level = 0;
  else 
    *batt_level = (((float) m_batt_lvl_in_milli_volts - BATT_VOLTAGE_MIN) / (BATT_VOLTAGE_MAX - BATT_VOLTAGE_MIN)) * 100;

  if (!nrf_drv_saadc_is_busy())
  {
    ret_code_t err_code = nrf_drv_saadc_buffer_convert(&m_adc_buf, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
  }
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         Function to handle ADC event
 *
 * @param[in]     p_evt     ADC event
 *
 * @attention     None
 *
 * @return        None
 */
static void m_saadc_event_handler(nrf_drv_saadc_evt_t const * p_evt)
{
  if (p_evt->type == NRF_DRV_SAADC_EVT_DONE)
  {
    nrf_saadc_value_t adc_result;

    adc_result = p_evt->data.done.p_buffer[0];

    m_batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);

    // NRF_LOG_INFO( "Battery mV: %d", m_batt_lvl_in_milli_volts);
  }
}

/* End of file -------------------------------------------------------- */
