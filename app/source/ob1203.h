/**
 * @file       ob1203.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      Driver support OB1203 (Blood Oximeter Sensor)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __OB1203_H
#define __OB1203_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "ob1203_defs.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief OB1203 status
 */
typedef enum
{
  OB1203_OK = 0x00,
  OB1203_ERR_PARAM,
  OB1203_ERR_I2C
}
ob1203_status_t;

/**
 * @brief OB1203 register bit struct
 */
typedef struct 
{
  uint8_t osc_trim;

  uint8_t ls_en;
  uint8_t ls_mode;
  uint8_t ls_sai;
  uint8_t sw_reset;

  uint8_t ppg_ps_en;
  uint8_t ppg_ps_mode;
  uint8_t ps_sai_en;
  uint8_t temp_en;

  uint8_t ls_int_en;
  uint8_t ls_var_mode;
  uint8_t ls_int_sel;

  uint8_t ps_int_en;
  uint8_t ps_logic_mode;
  uint8_t ppg_int_en;
  uint8_t a_full_int_en;

  uint8_t ps_persist;
  uint8_t ls_persist;

  uint32_t ls_thres_hi;
  uint32_t ls_thres_lo;

  uint16_t ps_thres_hi;
  uint16_t ps_thres_lo;

  uint16_t ps_current;

  uint16_t ir_current;
  uint16_t r_current;

  uint16_t ps_digital_can;

  uint8_t ppg_ps_gain;
  uint8_t ppg_led_setting;
  uint8_t ppg_alc_track;

  uint8_t ppg_pow_save;
  uint8_t led_flip;
  uint8_t sig_out;
  uint8_t diff;
  uint8_t alc;

  uint8_t ch1_can_ana;
  uint8_t ch2_can_ana;

  uint8_t ppg_avg;

  uint8_t ppg_pwidth;
  uint8_t ppg_freq;
  uint8_t ppg_rate;

  uint8_t bio_trim;

  uint8_t led_trim;
  uint8_t led1_trim;
  uint8_t led2_trim;

  uint8_t fifo_rollover_en;
  uint8_t fifo_afull_advance_warning;

  uint8_t ls_res;
  uint8_t ls_rate;
  uint8_t ls_gain;

  uint8_t ps_can_ana;
  uint8_t ps_pulses;

  uint8_t ps_pwidth;
  uint8_t ps_rate;

  uint8_t ps_avg_en;
  uint8_t ps_hys_level;

  uint8_t write_pointer;
  uint8_t read_pointer;
  uint8_t fifo_overflow;

  uint8_t led1_orig_trim;
  uint8_t led2_orig_trim;

  volatile bool ir_in_range;
  volatile bool r_in_range;
  volatile bool prev_in_range;
  volatile bool update_fast_mode;
  volatile bool update_current;
}
ob1203_reg_bit_t;

/**
 * @brief OB1203 sensor struct
 */
typedef struct 
{
  uint8_t device_address;  // I2C device address

  ob1203_reg_bit_t reg;    // Register bit struct

  // Read n-bytes from device's internal address <reg_addr> via I2C bus
  int  (*i2c_read) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

  // Write n-bytes from device's internal address <reg_addr> via I2C bus
  int  (*i2c_write) (uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
}
ob1203_t;


/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Initialize OB1203
 *
 * @param[in]     <me>  Pointer to handle of OB1203 module.
 *
 * @attention     None
 *
 * @return
 * - OB1203_OK
 * - OB1203_ERR_PARAM
 * - OB1203_ERR_I2C 
 */
ob1203_status_t ob1203_init(ob1203_t *me);
ob1203_status_t ob1203_reset(ob1203_t *me);
ob1203_status_t ob1203_get_status(ob1203_t *me, uint16_t *p_status);
ob1203_status_t ob1203_set_osc_trim(ob1203_t *me);
ob1203_status_t ob1203_set_main_config(ob1203_t *me);
ob1203_status_t ob1203_set_int_config(ob1203_t *me);
ob1203_status_t ob1203_set_ls_thres_h(ob1203_t *me);
ob1203_status_t ob1203_set_ps_thres_h(ob1203_t *me);
ob1203_status_t ob1203_set_ps_current(ob1203_t *me);
ob1203_status_t ob1203_set_ppg_current(ob1203_t *me);
ob1203_status_t ob1203_set_ppg_ps_gain_cfg(ob1203_t *me);
ob1203_status_t ob1203_set_ppg_can_ana(ob1203_t *me);
ob1203_status_t ob1203_set_ppg_avg_and_rate(ob1203_t *me);
ob1203_status_t ob1203_set_bio_trim(ob1203_t *me);
ob1203_status_t ob1203_set_led_trim(ob1203_t *me);
ob1203_status_t ob1203_set_dig_trim(ob1203_t *me);
ob1203_status_t ob1203_set_digital_can(ob1203_t *me);
ob1203_status_t ob1203_reset_fifo(ob1203_t *me);
ob1203_status_t ob1203_init_rbg(ob1203_t *me);
ob1203_status_t ob1203_init_ps(ob1203_t *me);
ob1203_status_t ob1203_init_ps_rbg(ob1203_t *me);
ob1203_status_t ob1203_init_hr(ob1203_t *me);
ob1203_status_t ob1203_init_spo2(ob1203_t *me);
uint32_t  ob1203_bytes_to_uint32(uint8_t *data, int start_byte) ;
uint32_t  ob1203_two_and_half_bytes_to_uint32(uint8_t *data, int start_byte) ;
uint8_t   ob1203_get_ls_data(ob1203_t *me, uint32_t *data);
uint8_t   ob1203_get_ps_data(ob1203_t *me, uint32_t *data);
uint8_t   ob1203_get_ps_ls_data(ob1203_t *me, uint32_t *data);
void      ob1203_get_fifo_info(ob1203_t *me, uint8_t *fifo_info);
void      ob1203_get_num_fifo_samples_available(ob1203_t *me, uint8_t *fifo_info, uint8_t *sample_info);
void      ob1203_get_fifo_samples(ob1203_t *me, uint8_t num_samples, uint8_t *fifo_data);
void      ob1203_parse_fifo_samples(ob1203_t *me, uint8_t num_samples, uint8_t *fifo_data, uint32_t *assembled_data);
uint8_t   ob1203_get_part_id(ob1203_t *me, uint8_t *data);
void      ob1203_do_agc(ob1203_t *me, uint32_t data, bool ch);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __OB1203_H

/* End of file -------------------------------------------------------- */
