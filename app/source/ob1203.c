/**
 * @file       ob1203.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      Driver support OB1203 (Blood Oximeter Sensor)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "ob1203.h"
#include "nrf_log.h"
#include "nrf_delay.h"

/* Private defines ---------------------------------------------------- */
#define OB1203_LOG_ENABLE           (false)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define OB1203_LOG(...)             \
  do {                              \
    if (OB1203_LOG_ENABLE)          \
      NRF_LOG_INFO(__VA_ARGS__);    \
  } while (0)

#define CHECK(expr, ret)            \
  do {                              \
    if (!(expr)) {                  \
      OB1203_LOG("%s", #expr);      \
      return (ret);                 \
    }                               \
  } while (0)

#define OB_CHECK(expr)              \
  do {                              \
    ob1203_status_t ret = (expr);   \
    if (OB1203_OK != ret) {         \
      OB1203_LOG("%s", #expr);      \
      return (ret);                 \
    }                               \
  } while (0)

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static ob1203_status_t m_ob1203_read_reg(ob1203_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);
static ob1203_status_t m_ob1203_write_reg(ob1203_t *me, uint8_t reg, uint8_t *p_data, uint32_t len);

/* Function definitions ----------------------------------------------- */
ob1203_status_t ob1203_init(ob1203_t *me)
{
  if ((me == NULL) || (me->i2c_read == NULL) || (me->i2c_write == NULL))
    return OB1203_ERR_PARAM;

  me->device_address = OB1203_I2C_ADDR;

  OB1203_LOG("ob1203_init success");
  return OB1203_OK;
}

ob1203_status_t ob1203_reset(ob1203_t *me)
{
  uint8_t buffer = SW_RESET;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_MAIN_CTRL_0, &buffer, 1));

  nrf_delay_ms(1000);
  OB1203_LOG("ob1203_reset success");
  return OB1203_OK;
}

ob1203_status_t ob1203_get_status(ob1203_t *me, uint16_t *p_status)
{
  uint8_t buffer[2];

  OB_CHECK(m_ob1203_read_reg(me, OB1203_REG_STATUS_0, buffer, 2));

  *p_status = (buffer[0] << 8 | buffer[1]);

  OB1203_LOG("ob1203_get_status success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_osc_trim(ob1203_t *me)
{
  uint8_t buffer;
  buffer = me->reg.osc_trim;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_OSC_TRIM, &buffer, 1));

  OB1203_LOG("ob1203_set_osc_trim success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_main_config(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = me->reg.ls_sai | me->reg.ls_mode | me->reg.ls_en;
  buffer[1] = me->reg.temp_en | me->reg.ps_sai_en | me->reg.ppg_ps_mode | me->reg.ppg_ps_en;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_MAIN_CTRL_0, buffer, 2));

  OB1203_LOG("ob1203_set_main_config success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_int_config(ob1203_t *me)
{
  uint8_t buffer[3];
  buffer[0] = me->reg.ls_int_sel | me->reg.ls_var_mode | me->reg.ls_int_en;
  buffer[1] = me->reg.a_full_int_en | me->reg.ppg_int_en | me->reg.ps_logic_mode | me->reg.ps_int_en;
  buffer[2] = me->reg.ls_persist | me->reg.ps_persist;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_INT_CFG_0, buffer, 3));

  OB1203_LOG("ob1203_set_int_config success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ls_thres_h(ob1203_t *me)
{
  uint8_t buffer[6];
  buffer[0] = (uint8_t) ( me->reg.ls_thres_hi & 0x000000FF);
  buffer[1] = (uint8_t) ((me->reg.ls_thres_hi & 0x0000FF00) >> 8);
  buffer[2] = (uint8_t) ((me->reg.ls_thres_hi & 0x00FF0000) >> 16);
  buffer[3] = (uint8_t) ( me->reg.ls_thres_lo & 0x000000FF);
  buffer[4] = (uint8_t) ((me->reg.ls_thres_lo & 0x0000FF00) >> 8);
  buffer[5] = (uint8_t) ((me->reg.ls_thres_lo & 0x00FF0000) >> 16);

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_LS_THRES_HI, buffer, 6));

  OB1203_LOG("ob1203_set_ls_thres_h success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ps_thres_h(ob1203_t *me)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)( me->reg.ps_thres_hi & 0x000FF);
  buffer[1] = (uint8_t)((me->reg.ps_thres_hi & 0xFF00) >> 8);
  buffer[2] = (uint8_t)( me->reg.ps_thres_lo & 0x000FF);
  buffer[3] = (uint8_t)((me->reg.ps_thres_lo & 0xFF00) >> 8);

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_THRES_HI, buffer, 4));

  OB1203_LOG("ob1203_set_ps_thres_h success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ps_current(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)( me->reg.ps_current & 0x00FF);
  buffer[1] = (uint8_t)((me->reg.ps_current & 0xFF00) >> 8);

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_LED_CURR, buffer, 2));

  OB1203_LOG("ob1203_set_ps_current success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ppg_current(ob1203_t *me)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)( me->reg.ir_current & 0x00FF);
  buffer[1] = (uint8_t)((me->reg.ir_current & 0xFF00) >> 8);
  buffer[2] = (uint8_t)( me->reg.r_current & 0x00FF);
  buffer[3] = (uint8_t)((me->reg.r_current & 0xFF00) >> 8);

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PPG_IRLED_CURR, buffer, 4));

  OB1203_LOG("ob1203_set_ppg_current success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ppg_ps_gain_cfg(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = me->reg.ppg_ps_gain | me->reg.ppg_led_setting | me->reg.ppg_alc_track;
  buffer[1] = me->reg.ppg_pow_save | me->reg.led_flip | me->reg.sig_out | me->reg.diff | me->reg.alc;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PPG_PS_GAIN, buffer, 2));

  OB1203_LOG("ob1203_set_ppg_ps_gain_cfg success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ppg_can_ana(ob1203_t *me)
{
  uint8_t buffer[1];
  buffer[0] = me->reg.ch1_can_ana | me->reg.ch2_can_ana;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PPG_CAN_ANA, buffer, 1));

  OB1203_LOG("ob1203_set_ppg_can_ana success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_ppg_avg_and_rate(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = me->reg.ppg_avg | 0x0A;
  buffer[1] = me->reg.ppg_pwidth | me->reg.ppg_freq | me->reg.ppg_rate;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PPG_AVG, buffer, 2));

  OB1203_LOG("ob1203_set_ppg_avg_and_rate success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_bio_trim(ob1203_t *me)
{
  uint8_t buffer[1];
  buffer[0] = me->reg.bio_trim;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_BIO_TRIM, buffer, 1));

  OB1203_LOG("ob1203_set_bio_trim success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_led_trim(ob1203_t *me)
{
  uint8_t buffer[1];
  buffer[0] = me->reg.led_trim;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_LED_TRIM, buffer, 1));

  OB1203_LOG("ob1203_set_led_trim success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_dig_trim(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = me->reg.led1_trim;
  buffer[1] = me->reg.led2_trim;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_DIG_LED1_TRIM, buffer, 2));

  OB1203_LOG("ob1203_set_dig_trim success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_digital_can(ob1203_t *me)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)( me->reg.ps_digital_can & 0x00FF);
  buffer[1] = (uint8_t)((me->reg.ps_digital_can & 0xFF00) >> 8);

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_CAN_DIG, buffer, 2));

  OB1203_LOG("ob1203_set_digital_can success");
  return OB1203_OK;
}

ob1203_status_t ob1203_set_fifo_config(ob1203_t *me)
{
  uint8_t buffer[1];
  buffer[0] = me->reg.fifo_rollover_en | me->reg.fifo_afull_advance_warning;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_FIFO_CFG, buffer, 1));

  OB1203_LOG("ob1203_set_fifo_config success");
  return OB1203_OK;
}

ob1203_status_t ob1203_reset_fifo(ob1203_t *me)
{
  uint16_t status;
  uint8_t buffer[5];
  buffer[0] = 0;
  buffer[1] = 0;
  buffer[2] = 0;
  buffer[3] = me->reg.temp_en | me->reg.ps_sai_en | me->reg.ppg_ps_mode | 0;
  buffer[4] = me->reg.temp_en | me->reg.ps_sai_en | me->reg.ppg_ps_mode | me->reg.ppg_ps_en;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_MAIN_CTRL_1, &buffer[3], 1));

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_FIFO_WR_PTR, buffer, 3));

  OB_CHECK(ob1203_get_status(me, &status));

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_MAIN_CTRL_1, &buffer[4], 1));

  OB1203_LOG("ob1203_reset_fifo success");
  return OB1203_OK;
}

ob1203_status_t ob1203_init_rbg(ob1203_t *me)
{
  uint8_t buffer[2];

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_LS_RES_RATE, buffer, 2));

  buffer[0] = me->reg.ls_res | me->reg.ls_rate;
  buffer[1] = me->reg.ls_gain;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_LS_RES_RATE, buffer, 2));

  OB_CHECK(ob1203_set_ls_thres_h(me));

  OB_CHECK(ob1203_set_int_config(me));

  me->reg.ppg_ps_en = PPG_PS_OFF;
  me->reg.ls_en     = LS_ON;

  OB_CHECK(ob1203_set_main_config(me));

  OB1203_LOG("ob1203_init_rbg success");
  return OB1203_OK;
}

ob1203_status_t ob1203_init_ps(ob1203_t *me)
{
  uint8_t buffer[2];

  OB_CHECK(ob1203_reset(me));

  OB_CHECK(ob1203_set_ps_current(me));

  buffer[0] = me->reg.ps_can_ana | me->reg.ps_pulses | 0x02;
  buffer[1] = me->reg.ps_pwidth | me->reg.ps_rate;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_CAN_PULSES, buffer, 2));

  OB_CHECK(ob1203_set_digital_can(me));

  buffer[0] = me->reg.ps_avg_en | me->reg.ps_hys_level;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_MOV_AVG_HYS, buffer, 1));

  OB_CHECK(ob1203_set_ps_thres_h(me));

  me->reg.ls_int_en = LS_INT_OFF;

  OB_CHECK(ob1203_set_ppg_ps_gain_cfg(me));

  me->reg.ls_en       = LS_OFF;
  me->reg.ppg_ps_en   = 1;
  me->reg.ppg_ps_mode = PS_MODE;

  OB_CHECK(ob1203_set_int_config(me));

  OB_CHECK(ob1203_set_main_config(me));

  OB1203_LOG("ob1203_init_ps success");
  return OB1203_OK;
}

ob1203_status_t ob1203_init_ps_rbg(ob1203_t *me)
{
  uint8_t buffer[2];

  OB_CHECK(ob1203_reset(me));

  buffer[0] = me->reg.ls_res | me->reg.ls_rate;
  buffer[1] = me->reg.ls_gain;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_LS_RES_RATE, buffer, 2));

  buffer[0] = me->reg.ps_can_ana | me->reg.ps_pulses | 0x02;
  buffer[1] = me->reg.ps_pwidth | me->reg.ps_rate;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_CAN_PULSES, buffer, 2));

  OB_CHECK(ob1203_set_digital_can(me));

  buffer[0] = me->reg.ps_avg_en | me->reg.ps_hys_level;

  OB_CHECK(m_ob1203_write_reg(me, OB1203_REG_PS_MOV_AVG_HYS, buffer, 1));

  OB_CHECK(ob1203_set_int_config(me));

  OB_CHECK(ob1203_set_ps_thres_h(me));

  OB_CHECK(ob1203_set_ps_current(me));

  OB_CHECK(ob1203_set_ls_thres_h(me));

  me->reg.ls_en       = LS_OFF;
  me->reg.ppg_ps_en   = PPG_PS_ON;
  me->reg.ppg_ps_mode = PS_MODE;

  OB_CHECK(ob1203_set_main_config(me));

  OB1203_LOG("ob1203_init_ps_rbg success");
  return OB1203_OK;
}

ob1203_status_t ob1203_init_hr(ob1203_t *me)
{
  OB_CHECK(ob1203_reset(me));

  me->reg.ps_int_en = PS_INT_OFF;
  me->reg.ls_en     = LS_OFF;

  OB_CHECK(ob1203_set_int_config(me));

  OB_CHECK(ob1203_set_ppg_ps_gain_cfg(me));

  OB_CHECK(ob1203_set_ppg_current(me));

  OB_CHECK(ob1203_set_ppg_can_ana(me));

  OB_CHECK(ob1203_set_ppg_avg_and_rate(me));

  OB_CHECK(ob1203_set_fifo_config(me));

  me->reg.ppg_ps_mode = HR_MODE;

  OB_CHECK(ob1203_set_main_config(me));

  OB1203_LOG("ob1203_init_hr success");
  return OB1203_OK;
}

ob1203_status_t ob1203_init_spo2(ob1203_t *me)
{
  OB_CHECK(ob1203_reset(me));

  me->reg.ps_int_en = PS_INT_OFF;
  me->reg.ls_en     = LS_OFF;

  OB_CHECK(ob1203_set_int_config(me));

  uint8_t buffer[1];
  m_ob1203_read_reg(me, OB1203_REG_PS_INT_CFG_1, buffer, 1);

  OB_CHECK(ob1203_set_ppg_ps_gain_cfg(me));

  OB_CHECK(ob1203_set_ppg_current(me));

  OB_CHECK(ob1203_set_ppg_can_ana(me));

  OB_CHECK(ob1203_set_ppg_avg_and_rate(me));

  OB_CHECK(ob1203_set_fifo_config(me));

  me->reg.ppg_ps_mode = SPO2_MODE;

  OB_CHECK(ob1203_set_main_config(me));

  OB1203_LOG("ob1203_init_spo2 success");
  return OB1203_OK;
}

uint32_t ob1203_bytes_to_uint32(uint8_t *data, int start_byte) 
{
  // Coverts a string of 3 bytes with LSB first into unsigned long MSB last
  return ((uint32_t)(data[start_byte + 2] & (0x3U))) << 16 |
          ((uint32_t)data[start_byte + 1]) << 8 |
          ((uint32_t)data[start_byte]);
}

uint32_t ob1203_two_and_half_bytes_to_uint32(uint8_t *data, int start_byte) 
{
  // Coverts a string of 3 bytes with LSB first into unsigned long MSB last
  uint32_t out = 0;
  out      |= (data[start_byte + 2] & 0x0F);
  out      <<= 8;
  out      |= (data[start_byte + 1] & 0x0F);
  out      <<= 8;
  out      |= (data[start_byte + 0] & 0x0F);

  return out;
}

uint8_t ob1203_get_ls_data(ob1203_t *me, uint32_t *data)
{
  uint8_t byte_data[21];

  m_ob1203_read_reg(me, OB1203_REG_STATUS_0, byte_data, 21);

  // Byte_data[0] is ps (not populated)
  data[1] = ob1203_two_and_half_bytes_to_uint32(byte_data, 4);                 // W
  data[2] = ob1203_two_and_half_bytes_to_uint32(byte_data, 7);                 // G
  data[3] = ob1203_two_and_half_bytes_to_uint32(byte_data, 10);                // B
  data[4] = ob1203_two_and_half_bytes_to_uint32(byte_data, 13);                // R
  data[5] = ob1203_two_and_half_bytes_to_uint32(byte_data, 16);                // C
  data[6] = (uint32_t)((byte_data[20] & 0x0F) << 8) | (uint32_t)byte_data[19]; // Temp data

  OB1203_LOG("ob1203_get_ls_data: %d", (byte_data[0] & LS_NEW_DATA) == 0x01 ? 1 : 0);

  return ((byte_data[0] & LS_NEW_DATA) == 0x01 ? 1 : 0);                       // Return 1 if new data or 0 if old data
}

uint8_t ob1203_get_ps_data(ob1203_t *me, uint32_t *data)
{
  uint8_t byte_data[4];

  m_ob1203_read_reg(me, OB1203_REG_STATUS_0, byte_data, 4);

  OB1203_LOG("%02x %02x %02x %02x\r\n", byte_data[0], byte_data[1], byte_data[2], byte_data[3]);

  data[0] = ((uint32_t)byte_data[3]) << 8 | ((uint32_t)byte_data[2]); // Ps data

  OB1203_LOG("ob1203_get_ps_data: %d", (byte_data[1] & PS_NEW_DATA) == 0x01 ? 1 : 0);

  return ((byte_data[1] & PS_NEW_DATA) == 0x01 ? 1 : 0);              // Return 1 if new data or 0 if old data
}

uint8_t ob1203_get_ps_ls_data(ob1203_t *me, uint32_t *data)
{
  uint8_t byte_data[21];

  m_ob1203_read_reg(me, OB1203_REG_STATUS_0, byte_data, 21);

  data[0] = ((uint32_t)byte_data[3]) << 8 | ((uint32_t)byte_data[2]);          // Ps
  data[1] = ob1203_two_and_half_bytes_to_uint32(byte_data, 4);                 // W
  data[2] = ob1203_two_and_half_bytes_to_uint32(byte_data, 7);                 // G
  data[3] = ob1203_two_and_half_bytes_to_uint32(byte_data, 10);                // B
  data[4] = ob1203_two_and_half_bytes_to_uint32(byte_data, 13);                // R
  data[5] = ob1203_two_and_half_bytes_to_uint32(byte_data, 16);                // C
  data[6] = (uint32_t)((byte_data[20] & 0x0F) << 8) | (uint32_t)byte_data[19]; // Temp data

  OB1203_LOG("ob1203_get_ps_ls_data: %d", (byte_data[0] & LS_NEW_DATA) == 0x01 ? 1 : 0);

  return ((byte_data[0] & LS_NEW_DATA) == 0x01 ? 1 : 0);                       // Return 1 if new data or 0 if old data
}

void ob1203_get_fifo_info(ob1203_t *me, uint8_t *fifo_info)
{
  OB1203_LOG("ob1203_get_fifo_info");

  m_ob1203_read_reg(me, OB1203_REG_FIFO_WR_PTR, fifo_info, 3);

  me->reg.write_pointer = fifo_info[0];
  me->reg.read_pointer  = fifo_info[1];
  me->reg.fifo_overflow = fifo_info[2];
}


void ob1203_get_num_fifo_samples_available(ob1203_t *me, uint8_t *fifo_info, uint8_t *sample_info)
{
 ob1203_get_fifo_info(me, fifo_info);

 uint8_t num_samples = me->reg.write_pointer;

 if (me->reg.write_pointer <= me->reg.read_pointer)
 {
   num_samples += 32;
 }

 num_samples -= me->reg.read_pointer;

 sample_info[0] = num_samples;        // Num HR samples
 sample_info[1] = (num_samples >> 1); // Num SpO2 samples
 sample_info[2] = fifo_info[2];

  OB1203_LOG("ob1203_get_num_fifo_samples_available_1: %d", num_samples);
}

void ob1203_get_fifo_samples(ob1203_t *me, uint8_t num_samples, uint8_t *fifo_data)
{
  OB1203_LOG("ob1203_get_fifo_samples: %d", num_samples);

  m_ob1203_read_reg(me, OB1203_REG_FIFO_DATA, fifo_data, 3 * num_samples);
}

void ob1203_parse_fifo_samples(ob1203_t *me, uint8_t num_samples, uint8_t *fifo_data, uint32_t *assembled_data)
{
  OB1203_LOG("ob1203_parse_fifo_samples");
  for (int n = 0; n < num_samples; n++)
  {
    assembled_data[n] = ob1203_bytes_to_uint32(fifo_data, 3 * n);
  }
}

uint8_t ob1203_get_part_id(ob1203_t *me, uint8_t *data)
{
  OB1203_LOG("ob1203_get_part_id");
  m_ob1203_read_reg(me, OB1203_REG_PART_ID, data, 1);

  return data[0];
}

void ob1203_do_agc(ob1203_t *me, uint32_t data, bool ch)
{
  OB1203_LOG("ob1203_do_agc: %d", ch);
  const  uint32_t tol1             = TOL1;
  const  uint32_t tol2             = TOL2;
  const  uint16_t in_range_persist = IN_RANGE_PERSIST;
  static uint16_t in_range[2]      = { 0, 0 };
  const  uint16_t max_current[2]   = { IR_MAX_CURRENT, R_MAX_CURRENT };
  const  uint16_t step             = STEP;
  const  uint32_t target_counts[2] = { TARGET_COUNTS, TARGET_COUNTS };

  if (data > target_counts[ch] + ((in_range[ch] > in_range_persist) ? tol2 : tol1)) // Too high
  {
    if (data > (target_counts[ch] + tol2))
    {
      in_range[ch] = 0;
    }

    if ((ch ? me->reg.r_current : me->reg.ir_current) > step)
    {
      if (ch)
      {
        me->reg.r_current -= step;
      }
      else
      {
        me->reg.ir_current -= step;
      }
      me->reg.update_current = 1;
    }
    OB1203_LOG("Too hight");
  }
  else if (data < target_counts[ch] - ((in_range[ch] > in_range_persist) ? tol2 : tol1)) // Too low
  {
    if (data < (target_counts[ch] - tol2))
    {
      in_range[ch] = 0;
    }

    if ((ch ? me->reg.r_current : me->reg.ir_current) + step < max_current[ch]) // No need to go to full current
    {
      if (ch)
      {
        me->reg.r_current += step;
      }
      else
      {
        me->reg.ir_current += step;
      }
      me->reg.update_current = 1;
    }
    OB1203_LOG("Too low");
  }
  else
  {
    if ((data > (target_counts[ch] - tol1)) && (data < (target_counts[ch] + tol1))) // Just right
    {
      if (in_range[ch] <= in_range_persist)
      {
        in_range[ch]++;
      }
    }
    OB1203_LOG("Just right");
  }

  if (in_range[ch] > in_range_persist)
  {
    if (ch)
    {
      me->reg.r_in_range = 1;
    }
    else
    {
      me->reg.ir_in_range = 1;
    }
    OB1203_LOG("In range");
  }
  else
  {
    if (ch)
    {
      me->reg.r_in_range = 0;
    }
    else
    {
      me->reg.ir_in_range = 1;
    }
  }

  if ( me->reg.prev_in_range && !(me->reg.ir_in_range && me->reg.r_in_range))
  {
    me->reg.prev_in_range    = 0;
    me->reg.update_fast_mode = 1;
    me->reg.a_full_int_en    = AFULL_INT_OFF;
    me->reg.ppg_int_en       = PPG_INT_ON;
  }
  else if (!me->reg.prev_in_range && me->reg.ir_in_range && me->reg.r_in_range)
  {
    me->reg.prev_in_range    = 1;
    me->reg.update_fast_mode = 1;
    me->reg.a_full_int_en    = AFULL_INT_ON;
    me->reg.ppg_int_en       = PPG_INT_OFF;
  }

  OB1203_LOG("r_in_range: %d", me->reg.r_in_range);
  OB1203_LOG("ir_in_range: %d", me->reg.ir_in_range);

}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         OB1203 read register
 *
 * @param[in]     me      Pointer to handle of OB1203 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - OB1203_OK
 * - OB1203_ERR_I2C
 */
static ob1203_status_t m_ob1203_read_reg(ob1203_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_read(me->device_address, reg, p_data, len), OB1203_ERR_I2C);

  return OB1203_OK;
}

/**
 * @brief         OB1203 write register
 *
 * @param[in]     me      Pointer to handle of OB1203 module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 * @param[in]     len     Data length
 *
 * @attention     None
 *
 * @return
 * - OB1203_OK
 * - OB1203_ERR_I2C
 */
static ob1203_status_t m_ob1203_write_reg(ob1203_t *me, uint8_t reg, uint8_t *p_data, uint32_t len)
{
  CHECK(0 == me->i2c_write(me->device_address, reg, p_data, len), OB1203_ERR_I2C);

  return OB1203_OK;
}

/* Undefine macros ---------------------------------------------------------- */
#undef CHECK
#undef OB_CHECK
/* End of file -------------------------------------------------------- */
