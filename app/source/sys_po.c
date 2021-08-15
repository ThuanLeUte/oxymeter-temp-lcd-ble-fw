/**
 * @file       sys_po.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      Sytem module to handle Pulse Oximeter (PO)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "ob1203.h"
#include "spo2.h"
#include "sys_po.h"
#include "kalman.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"

/* Private defines ---------------------------------------------------- */
#define PROX_MODE         (0)
#define BIO_MODE          (1)
#define MAX_LOW_SAMPLES   (30)
#define PROX_DELAY        (100)

/* Constant variables ------------------------------------------------- */
const uint32_t BIO_THRESHOLD = 100000;
const uint8_t meas_ps        = 1;
const uint8_t ppg2           = 1;     // 0 for HR, 1 for SpO2
const uint8_t redAGC         = 1;
const uint8_t irAGC          = 1;

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern kalman_t kalman_filters[NUM_KALMAN_FILTERS];
extern volatile uint16_t sample_count;

/* Private variables -------------------------------------------------- */
static uint16_t temp_br = 0;

static volatile unsigned char uart2_busy_flag;
static volatile uint8_t uart2_receive_complete_flag;
static volatile uint8_t m_samples_ready = 0;
static volatile uint8_t m_just_woke_up  = 0;
static uint8_t  mode                    = PROX_MODE;  // Start in prox mode, then switch to HR mode when we detect proximity

static uint8_t do_part1;
static uint8_t do_part2;
static uint8_t do_part3;
static uint8_t samples_processed;
uint8_t get_last_samples                 = 0;
uint8_t sample_log                       = 0;
static  volatile uint8_t num_low_samples = 0;

static ob1203_t m_ob1203;

/* Private function prototypes ---------------------------------------- */
static void m_sys_po_config(void);
static void m_sys_po_switch_mode(uint8_t prox_bio_mode);
static void m_sys_po_get_sensor_data(void);
static void m_sys_po_data_event(void);
static void m_sys_po_prox_event(void);

static ob1203_status_t m_sys_po_init(void);
static void (*p_intr_event)(void)   = NULL;

/* Function definitions ----------------------------------------------- */
void sys_po_main(void)
{
  m_sys_po_init();
  init_kalman(kalman_filters);  // Before SPO2_init()
  spo2_init();                  // After init_kalman()

  m_sys_po_config();            // Do the OB1203 configuration now

  p_intr_event   = &m_sys_po_prox_event; // Set an interrupt on the falling edge to wake from prox mode

  spo2_get_sum_squares();

  m_ob1203.reg.ps_int_en = PS_INT_OFF;
  ob1203_set_int_config(&m_ob1203);
  m_ob1203.reg.ppg_ps_en = PPG_PS_OFF;
  ob1203_set_main_config(&m_ob1203);
  m_ob1203.reg.ps_int_en = PS_INT_ON;
  ob1203_set_int_config(&m_ob1203);
  m_ob1203.reg.ppg_ps_en = PPG_PS_ON;
  ob1203_set_main_config(&m_ob1203);

  display_spo2 = 1;
  display_hr   = 2;

  NRF_LOG_INFO("Main loop");
  while (1)
  {
    // Main program loop
    if (NRF_LOG_PROCESS() == false)
    {
      nrf_pwr_mgmt_run();
      if (mode == PROX_MODE)
      {
        if (m_just_woke_up)
        {
          NRF_LOG_INFO("m_just_woke_up");
          mode           = BIO_MODE;
          p_intr_event   = &m_sys_po_data_event;  // Attach interrupt to data events
          m_just_woke_up = 0;
          m_sys_po_switch_mode(mode);             // Starts in PPG fast mode
        }
      }
      else
      {

        while(sample_log < INTERVAL)
        {
          if (mode == BIO_MODE)
          {
            if (m_samples_ready)
            { 
              // Only read data if available (m_samples_ready is asserted by ISR and cleard by m_sys_po_get_sensor_data)
              m_sys_po_get_sensor_data();
              samples_processed = true;
              sample_log += 10;
              if (get_last_samples == 1)
              {
                get_last_samples = 0;
                do_part1 = 0;
              }
              else if (do_part1 == 0)
              {
                do_part1 = 1;
              }
            }
          }
          else
          {
            break; // Exit loop and go to sleep
          }
        }

        if ((do_part1 == 1) && m_ob1203.reg.a_full_int_en && (samples_processed == true))
        {
          spo2_do_algorithm_part1();
          samples_processed = 0;
          do_part1          = 2;
          do_part2          = 1;
        }
        if (do_part2 && m_ob1203.reg.a_full_int_en && (samples_processed == true))
        {
          // If we are in bio slow read mode and we haven't done part 2 yet
          spo2_do_algorithm_part2();

          do_part2          = 0;
          do_part3          = 1;
          samples_processed = 0;
        }
        if ((do_part3 == true) && (samples_processed == true))
        {
          spo2_do_algorithm_part3(); // Triangle filter and peak counting
          spo2_do_algorithm_part4(); // rr filter selector

          do_part3          = 0;
          get_last_samples  = 1;
          samples_processed = 0;
          temp_br           = (uint16_t)(breathing_rate1f >> FIXED_BITS);

          NRF_LOG_ERROR("SPO2: %d", (uint8_t)(display_spo2 / 10));
          NRF_LOG_ERROR("Heart rate : %d", (uint8_t)display_hr);
          NRF_LOG_ERROR("Breath rate : %d", temp_br);

          if (0 != display_hr)
          {
            NRF_LOG_ERROR("Send data to BLE");
            sys_po_blood_oxygen_notify((uint8_t)(display_spo2 / 10));
            sys_po_heart_rate_notify((uint8_t)display_hr);
          }
        }
      }
      sample_log = 0;
    }
  }
}

/* Private function definitions --------------------------------------- */
void m_sys_po_data_event(void)
{
  m_samples_ready = 1;
}

void m_sys_po_prox_event(void)
{
  m_just_woke_up = 1;
}

ob1203_status_t m_sys_po_init(void)
{
  m_ob1203.device_address = OB1203_I2C_ADDR;
  m_ob1203.i2c_read       = bsp_i2c_read;
  m_ob1203.i2c_write      = bsp_i2c_write;

  return ob1203_init(&m_ob1203);
}

void m_sys_po_config(void) 
{
  // Temperature sensor settings (hidden registers)
  m_ob1203.reg.temp_en = TEMP_OFF;
  m_ob1203.reg.ls_en   = LS_OFF;

  // PS and PPG settings
  m_ob1203.reg.ps_sai_en      = PS_SAI_OFF;
  m_ob1203.reg.ps_pulses      = PS_PULSES(3);
  m_ob1203.reg.ps_pwidth      = PS_PWIDTH(1);
  m_ob1203.reg.ps_rate        = PS_RATE(5);
  m_ob1203.reg.ps_avg_en      = PS_AVG_OFF;
  m_ob1203.reg.ps_can_ana     = PS_CAN_ANA_0;
  m_ob1203.reg.ps_digital_can = 0;
  m_ob1203.reg.ps_hys_level   = 0;

  if (meas_ps)
  {
    m_ob1203.reg.ps_current = 0x0A8;
  }
  else
  {
    m_ob1203.reg.ps_current = 0x000;
  }

  m_ob1203.reg.ps_thres_hi = 0x3A98; // 15,000 decimal
  m_ob1203.reg.ps_thres_lo = 0x00;

  // Interrupts
  m_ob1203.reg.ls_int_sel    = LS_INT_SEL_W;
  m_ob1203.reg.ls_var_mode   = LS_THRES_INT_MODE;
  m_ob1203.reg.ls_int_en     = LS_INT_OFF;
  m_ob1203.reg.ppg_ps_en     = PPG_PS_ON;
  m_ob1203.reg.ps_logic_mode = PS_INT_READ_CLEARS;
  m_ob1203.reg.ps_int_en     = PS_INT_OFF;          // Turn it on later after display boot.
  m_ob1203.reg.ls_persist    = LS_PERSIST(2);
  m_ob1203.reg.ps_persist    = PS_PERSIST(2);

  // BIO SETTINGS
  // Int
  m_ob1203.reg.a_full_int_en = AFULL_INT_OFF;
  m_ob1203.reg.ppg_int_en    = PPG_INT_ON;

  // PPG
  m_ob1203.reg.ir_current = 0x1AF;  // Max 1023. 3FF
  if (ppg2)
  {
    m_ob1203.reg.r_current = 0x1AF; // Max 511. 1FF
  }
  else
  {
    m_ob1203.reg.r_current = 0;
  }

  m_ob1203.reg.ppg_ps_gain  = PPG_PS_GAIN_1;
  m_ob1203.reg.ppg_pow_save = PPG_POW_SAVE_OFF;
  m_ob1203.reg.led_flip     = LED_FLIP_OFF;
  m_ob1203.reg.ch1_can_ana  = PPG_CH1_CAN(0);
  m_ob1203.reg.ch2_can_ana  = PPG_CH2_CAN(0);
  m_ob1203.reg.ppg_avg      = PPG_AVG(4);        // 2^n averages 100Hz


  m_ob1203.reg.ppg_rate                   = PPG_RATE(1);                  // PPG_RATE(1);
  m_ob1203.reg.ppg_pwidth                 = PPG_PWIDTH(3);                // PPG_PWIDTH(3);
  m_ob1203.reg.ppg_freq                   = PPG_FREQ_PRODUCTION;          // Sets the data collection rate to multiples of 50Hz.
  m_ob1203.reg.bio_trim                   = 3;                            // Sax 3 --this dims the ADC sensitivity, but reduces noise
  m_ob1203.reg.led_trim                   = 0x00;                         // Can use to overwrite trim setting and max out the current
  m_ob1203.reg.ppg_led_setting            = PPG_LED_SETTLING(2);          // Hidden regstier for adjusting LED setting time (not a factor for noise)
  m_ob1203.reg.ppg_alc_track              = PPG_ALC_TRACK(2);             // Hidden register for adjusting ALC track and hold time (not a factor for noise)
  m_ob1203.reg.diff                       = DIFF_ON;                      // Hidden register for turning off subtraction of residual ambient light after ALC
  m_ob1203.reg.alc                        = ALC_ON;                       // Hidden register for turning off ambient light cancelleation track and hold circuit
  m_ob1203.reg.sig_out                    = SIGNAL_OUT;                   // Hidden register for selecting ambient sample or LED sample if DIFF is off
  m_ob1203.reg.fifo_rollover_en           = FIFO_ROLL_ON;
  m_ob1203.reg.fifo_afull_advance_warning = AFULL_ADVANCE_WARNING(0x0C);  // Balance early warning versus large sample count

  // Run initialization according to user compile settings
  uint8_t reg_data[2];
  bsp_i2c_read(OB1203_I2C_ADDR, OB1203_REG_DIG_LED1_TRIM, reg_data, 2);
  m_ob1203.reg.led1_orig_trim = reg_data[0];
  m_ob1203.reg.led2_orig_trim = reg_data[1];

  if (mode == BIO_MODE)
  {
    ppg2 ? ob1203_init_spo2(&m_ob1203) : ob1203_init_hr(&m_ob1203);
  }
  else
  {
    m_ob1203.reg.ppg_int_en = PPG_INT_OFF;
    ob1203_init_ps(&m_ob1203);
  }
}

void m_sys_po_switch_mode(uint8_t prox_bio_mode)
{
  if (prox_bio_mode == BIO_MODE)
  {
    m_ob1203.reg.a_full_int_en = AFULL_INT_OFF;
    m_ob1203.reg.ppg_int_en    = PPG_INT_ON;
  
    spo2_init();                 // Completely reset the algorithm
    ob1203_init_spo2(&m_ob1203);

    m_ob1203.reg.led1_trim = 0x00; // Overwrite default prox sensor trim--not used for PPG mode
    m_ob1203.reg.led2_trim = 0x00; // Overwrite default prox sensor trim--not used for PPG mode

    ob1203_set_dig_trim(&m_ob1203);

    samples_processed = 0;
    do_part1          = 0;
    do_part2          = 0;
    do_part3          = 0;
    num_low_samples   = 0;
  }
  else
  {
    m_ob1203.reg.a_full_int_en = AFULL_INT_OFF;
    m_ob1203.reg.ppg_int_en    = PPG_INT_OFF;
    m_ob1203.reg.ps_int_en     = PS_INT_ON;
    m_ob1203.reg.ppg_ps_mode   = PS_MODE;
    m_ob1203.reg.ppg_ps_en     = PPG_PS_OFF;

    ob1203_set_main_config(&m_ob1203); // Dan-apparently can't just change PPG to PS without turning it off first.
    ob1203_init_ps(&m_ob1203);

    m_ob1203.reg.led1_trim = m_ob1203.reg.led1_orig_trim; // Overwrite default prox sensor trim--not used for PPG mode
    m_ob1203.reg.led2_trim = m_ob1203.reg.led2_orig_trim; // Overwrite default prox sensor trim--not used for PPG mode
    ob1203_set_dig_trim(&m_ob1203);
  }
}

void m_sys_po_get_sensor_data(void)
{
  const uint8_t maxSamples2Read = 16; // FIFO samples, e.g. 4 samples * 3 bytes = 12 bytes (or 2 SpO2 samples) 16 samples is the entire SpO2 buffer.
  uint8_t fifoBuffer[16 * 6];         // $AE constant
  uint32_t ppgData[16 * 2];           // $AE constant
  uint8_t sample_info[3];
  uint8_t fifo_reg_data[3];
  uint8_t overflow      = 0;
  uint8_t samples2Read  = 0;
  uint8_t do_reset_fifo = 0;

  if (m_ob1203.reg.a_full_int_en)
  {
    // Slow mode--find out how many samples in buffer
    ob1203_get_num_fifo_samples_available(&m_ob1203, fifo_reg_data, sample_info);          // Read the samples fifo registers and figure out how many samples are left
    samples2Read = (sample_info[1] > maxSamples2Read) ? maxSamples2Read : sample_info[1];  // Limit the number of samples to the maximum
    overflow     = sample_info[2];                                                         // Add by guan 20200122
  }
  else
  {
    samples2Read = 1;  // Read one sample
    overflow     = 0;
  }
  ob1203_get_fifo_samples(&m_ob1203, samples2Read << 1, fifoBuffer);
  ob1203_parse_fifo_samples(&m_ob1203, samples2Read << 1, fifoBuffer, ppgData);

  if (m_ob1203.reg.ir_in_range && m_ob1203.reg.r_in_range)
  {
    uart2_busy_flag = 0;
    for (int n = 0; n < (overflow >> 1); n++)
    {
      spo2_add_sample(ppgData[0], ppgData[1]); // Duplicate oldest data to deal with missing (overwritten) samples
      if (sample_count < ARRAY_LENGTH)
        sample_count++; // Number of samples in buffer
      missed_sample_count++;
    }
    for (int n = 0; n < samples2Read; n++)
    {
      // Add samples
      spo2_add_sample(ppgData[2 * n], ppgData[2 * n + 1]); // Add data to sample buffer when data is in range

      // Send notify
      if (n == 1) // Just send 1 sample
      {
        sys_po_r_data_notify((uint32_t)ppgData[2 * n + 1]);
        sys_po_ir_data_notify((uint32_t)ppgData[2 * n]);
      }

      if (sample_count < ARRAY_LENGTH)
        sample_count++; // Number of samples in buffer
      read_sample_count++;
    }
  }
  else
  {
    sample_count = 0;
  }

  if (irAGC)
  {
    ob1203_do_agc(&m_ob1203, ppgData[2 * (samples2Read - 1)], 0);     // Use the most recent sample in the FIFO
  }

  if (ppg2 && redAGC)
  {
    ob1203_do_agc(&m_ob1203, ppgData[2 * (samples2Read - 1) + 1], 1); // Use the most recent sample in the FIFO
  }

  if (m_ob1203.reg.update_fast_mode || m_ob1203.reg.update_current)
  {
    do_reset_fifo = 1;
  }

  if (m_ob1203.reg.update_fast_mode)
  {
    ob1203_set_int_config(&m_ob1203);
    m_ob1203.reg.update_fast_mode = 0;
    if (m_ob1203.reg.ppg_int_en == 0)
    {
      reset_kalman_hr   = 1;  // Reset the Kalman filter for SpO2
      reset_kalman_spo2 = 1;  // Reset the Kalman filter for SpO2
    }
  }
  if (m_ob1203.reg.update_current)
  {
    ob1203_set_ppg_current(&m_ob1203);
    m_ob1203.reg.update_current = 0;
  }
  if (do_reset_fifo)
  {
    ob1203_reset_fifo(&m_ob1203);
    do_reset_fifo = 0;
  }

  // Switch modes if no signal
  if (ppgData[2 * (samples2Read - 1)] < BIO_THRESHOLD)
  {
    num_low_samples++;
  }
  else
  {
    num_low_samples = 0;
  }

  if (num_low_samples >= MAX_LOW_SAMPLES)
  {
    mode = PROX_MODE;
    p_intr_event = &m_sys_po_prox_event; // Set an interrupt on the falling edge

    m_sys_po_switch_mode(mode);
    m_just_woke_up = 0;
  }
  m_samples_ready = 0;

}

void bsp_intr_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  p_intr_event();
}

/* End of file -------------------------------------------------------- */
