/**
 * @file       spo2.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-25
 * @author     Thuan Le
 * @brief      Handle calculate heart rate and blood oxigen
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "spo2.h"
#include "ob1203.h"
#include "kalman.h"
#include "bsp.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
uint16_t data_ptr;
uint16_t prev_data_ptr;
int32_t mean1f[2];
float R;                                 // R value used for SpO2 calculation
uint32_t rms1f[2];                       // Rms value in fixed precision
float perfusion_index[2];

uint16_t current_spo21f;                 // Most recent SpO2 calculation in percent and fixed precision
uint16_t current_hr1f;                   // Most recent heart rate value in BPM and fixed precision
uint16_t avg_hr1f;                       // Average heart rate determined by Kalman filter to output to user (fixed precision)
uint16_t avg_spo21f;                     // Average Spo2 value determined by Kalman filter to output ot user (fixed precision)
volatile uint16_t sample_count;          // Keeps track of whether the sample buffer is full or not. If full then alg runs.
volatile uint32_t read_sample_count;     // Keeps track of total number of samples collected
volatile uint32_t missed_sample_count;
uint32_t prev_sample_count;
uint32_t prev_missed_samples;
uint16_t read_samples;
uint16_t missed_samples;
int32_t alg_start_sample_count;          // Records the number most recent sample when the algorithm starts
int32_t prev_alg_start_sample_count;     // Remembers the sample count start position of the previous alg run
int16_t b2b1f;                           // Beat 2 beat time in # of samples
int16_t ppg_falltime;                    // Ppg fall time in # of samples
volatile uint8_t reset_kalman_hr;        // Flag to reset kalman filter for HR
volatile uint8_t reset_kalman_spo2;      // Flag to reset kalman filter for SpO2
uint16_t p2_start_time;                  // Tracking when the algorithm starts to check for timeout
uint16_t display_spo2;                   // Value output to display
uint16_t display_hr;                     // Value output to display
int32_t fit_correl;                      // Estimated correlation value from quadratic fit function.
uint8_t samples2avg;                     // Length of moving average filter for heart rate calculations
uint16_t rr_time1f;                      // Fixed precision output from peak find beat to beat time --FIGURE OUT A WAY TO PUBLISH THIS DATA. IT IS ASYNCHROUS.
int32_t peak_amp;                        // From m_spo2_simple_peak_find and get_peak_height
int32_t hr_data_buffer[MAX_BREATH_ARRAY_LENGTH];
uint8_t hr_data_buffer_ind;
uint32_t breathing_rate1f;               // Breaths per minute with fixed precision
int16_t iir_avg[NUM_RR_FILTERS];
int16_t iir_rms[NUM_RR_FILTERS];
int16_t iir_rms_smooth[NUM_RR_FILTERS];
uint8_t rr_filter_lens[NUM_RR_FILTERS];  // Values are set in the class init function
int alg_count;
int period1f;
uint8_t res_avg;                         // For resetting the 30sec RR average (after a finger was removed, e.g.)
int16_t final_offset1f;
uint32_t first_data;

/* Private variables -------------------------------------------------- */
static uint16_t ds_start;                // Point in the array where the downsampled data will start;
static uint16_t downsampled_array_length;
static uint16_t downsampled_max_centered_index;

static uint16_t max_offset;              // Determines minimum heart rate
static uint16_t min_offset;              // Determines maximum heart rate
static uint16_t indx;
static uint16_t indx1;
static uint8_t do_rr_alg;

static uint8_t fast;

static uint8_t prev_valid;               // Did the previous algorithm yeild a valid heart rate?
static uint16_t final_offset;            // Integer value nearest the peak
static int32_t final_correl;             // Value of the correlation near the peak
static uint16_t offset_guess;            // Trial point to find the correlation peak
static uint16_t prev_offset;             // Previous values to use in the case of algorithm bonk

// Declaring these large arrays in global scope to limit stack usage
static int16_t idx2[((ARRAY_LENGTH - 1) >> DOWNSAMPLE_BITS) + 1];
static int16_t AC1f[ARRAY_LENGTH];                  // Including 4 bits fixed point after subtracting local mean (max AC amplitude (half of P2P signal must be less than 2^(15-4) = 2^11 = 2048)
static int16_t raw_data[2][ARRAY_LENGTH];
static int32_t ac_filtered[ARRAY_LENGTH];
static int16_t filter[MAX_TRIANGLE_FILTER_LENGTH];  // Used both for ppg and breath FIR filters
static int32_t rr_filter[MAX_BREATH_FILTER_SPAN];
static int32_t lin_buffer[MAX_BREATH_ARRAY_LENGTH];
static int32_t detrended[MAX_BREATH_FILTER_SPAN];   // Int32_t fom[NUM_RR_FILTERS]; //figure of merit = amp/rms
static uint8_t baseline_sign_changes[NUM_RR_FILTERS];
static uint8_t zero_crossings[NUM_RR_FILTERS];
static int16_t badness[NUM_RR_FILTERS];
static int32_t systole;                  // Systole peak amplitude (single sided, not peak height difference from diastole)
static uint8_t max_fom;                  // Min badness

static uint8_t filter_tracker;
static uint8_t current_filter;
static uint8_t systole_found;
static uint8_t char_array_iir_initialized;

static int display_avg_buffer[DISPLAY_NUM2AVG];
static int display_avg_cnt = 0;
static int display_avg_buffer_ind = 0;

static int32_t sum_squares;
static int32_t sum_quads;
static uint32_t rr_current_monitor1f = 0;

/* Private function prototypes ---------------------------------------- */
static void m_spo2_get_idx(void);
static void m_spo2_copy_data(uint8_t channel);
static void m_spo2_get_dc(void);
static void m_spo2_get_rms(void);
static void m_spo2_calc_r(void);
static void m_spo2_calc_spo2(void);
static void m_spo2_calc_hr(void);
static void m_spo2_fine_search(int16_t *x, uint16_t len, uint16_t start_offset, int32_t start_correl, uint16_t search_step);
static void m_spo2_peak_find(uint16_t *x0, int32_t *y, int16_t *x_fit1f, int32_t *y_fit) ;
static void m_spo2_simple_peak_find(int32_t *y, int16_t *x_fit1f, int32_t *yfit, uint16_t x_center, int16_t step);
static void m_spo2_avg_n_samples(int16_t *x, int16_t len, uint8_t number2avg);
static void m_spo2_findminmax32(int32_t* data, uint16_t start_ind, uint16_t stop_ind, int32_t* c, int* extreme, int* type); 
static void m_spo2_peak2peak(int32_t* array);
static void m_spo2_triangle_filter_new_arr(int16_t* arr, uint8_t arr_len, int32_t* new_arr, uint8_t num_slopes, int16_t* slopes, uint8_t* lengths, int16_t start_val, uint8_t subtract_avg); //returns filtered values in long array, unnormalized (max resolution).
static void m_spo2_characterize_array_iir(uint8_t arr_num, int16_t new_val, int16_t* iir_avg, int16_t *iir_rms, int16_t* iir_rms_smooth);

static int16_t m_spo2_div2n(int16_t num,uint8_t bits); 
static int32_t m_spo2_div2n32(int32_t num, uint8_t bits); 
static int32_t m_spo2_corr(int16_t *x, uint16_t len, uint16_t offset);
static uint8_t m_spo2_count_sign_changes(int32_t* arr, uint8_t len);
static uint8_t m_spo2_count_zero_crossings(int32_t* arr, uint8_t len);
static uint8_t m_spo2_check_4_max(int16_t *x, uint16_t len,uint16_t start_offset, int32_t start_correl);
static uint8_t m_spo2_find_max_corr(int16_t *x, uint16_t max_length, uint16_t offset_guess);
static int m_spo2_get_direction(int32_t data1, int32_t data2);
static int m_spo2_sign(int32_t val);

/* Function definitions ----------------------------------------------- */

void spo2_init(void)
{
  // Triangle (works better than flat top in matlab sims)
  const uint8_t temp_arr1[] = {12, 16, 20, 24, 28};

  min_offset               = MIN_OFFSET;
  max_offset               = MAX_OFFSET;
  prev_valid               = 0;
  data_ptr                 = 0;
  final_offset             = 0;
  reset_kalman_hr          = 1;
  reset_kalman_spo2        = 1;
  avg_hr1f                 = 0;
  avg_spo21f               = 0;
  samples2avg              = MAX_FILTER_LENGTH / 2;
  downsampled_array_length = ((ARRAY_LENGTH - 1) >> DOWNSAMPLE_BITS);
  ds_start                 = 0;
  alg_count                = 0;

  for (int n = 0; n < NUM_RR_FILTERS; n++)
  {
    rr_filter_lens[n] = temp_arr1[n];
  }

  do_rr_alg                  = 0;
  char_array_iir_initialized = 0;

  // These variables are for counting number of samples collected between runs of the algorithm.
  sample_count                = 0;
  alg_start_sample_count      = 0;
  prev_alg_start_sample_count = 0;
  prev_sample_count           = 0;
  prev_missed_samples         = 0;
  read_sample_count           = 0;
  missed_sample_count         = 0;
  ppg_falltime                = 0;

  if (downsampled_array_length & 0x0001)
  {
    // Odd case
    ds_start = 1 << (DOWNSAMPLE_BITS - 1);
  }
  else
  {
    // Even case
    downsampled_array_length++;
  }

  downsampled_max_centered_index = (downsampled_array_length - 1) >> 1;
  hr_data_buffer_ind             = 0;
  current_filter                 = 0;
  filter_tracker                 = 0;
  res_avg                        = 1;
  reset_kalman(corr_filt);
  reset_kalman(hr_filt);
  reset_kalman(spo2_filt);
  reset_kalman(rr_filt);
  systole_found = 0;
}

void spo2_get_sum_squares(void)
{
  m_spo2_get_idx();
  sum_squares = 0;
  sum_quads   = 0;

  // Do the single-sided sum of squares and fourth power terms
  for (int n = 0; n < downsampled_max_centered_index; n++)
  {
    sum_squares += idx2[n];
    sum_quads   += idx2[n] * idx2[n];
  }
  sum_squares = labs(sum_squares << 1);                                                         // Make the sum double sided
  sum_quads   = labs(sum_quads << 1) - sum_squares * sum_squares / (downsampled_array_length);  // Make the sum double sided and subtract the square term
}

void spo2_do_algorithm_part1(void)
{ 
  alg_start_sample_count = read_sample_count;
  read_samples           = read_sample_count - prev_sample_count;
  prev_sample_count      = read_sample_count;
  missed_samples         = missed_sample_count - prev_missed_samples;
  prev_missed_samples    = missed_sample_count;
  indx1                  = data_ptr;                                   // Capture the current data pointer value

  if (sample_count >= ARRAY_LENGTH)
  {
    m_spo2_get_rms(); // Subtract residual slopes and calculate RMS
  }
}

void spo2_do_algorithm_part2(void)
{
  uint8_t num_slopes    = 3;
  int16_t slopes[3]     = { 2, 0, -2 };
  uint8_t subtract_avg  = 0;
  uint8_t lengths[3];
  p2_start_time = bsp_time_now();

  if (kalman_filters[corr_filt].kalman_avg > 0)
  {                                                                                                                 // Changed this to symmetrical triangle--it filters dicrotic notch better than asymmetric matched filter
    lengths[0] = (uint8_t)(((kalman_filters[corr_filt].kalman_avg >> FIXED_BITS) >> 1) + MIN_TRIANGLE_HALF_LENGTH); // Rising slope portion (50%) + offset MIN_TRIANGLE_HALF_LENGTH to more strongly filter fast signals which are more noise sensitive
    lengths[1] = 1;
    lengths[2] = (uint8_t)((((kalman_filters[corr_filt].kalman_avg >> FIXED_BITS) >> 1) - 1) + MIN_TRIANGLE_HALF_LENGTH); // Falling slope portion (50%)
  }

  uint8_t ppg_fir_filter_length = lengths[0] + lengths[1] + lengths[2];
  if (ppg_fir_filter_length > MAX_OFFSET)
  {
    ppg_fir_filter_length = MAX_OFFSET;
  }
  int16_t start_val = -(lengths[0] - 1);

  offset_guess = DEFAULT_GUESS;

  if (alg_start_sample_count >= ARRAY_LENGTH)
  {
    // Original algorithm part 2 (RMS-based)
    m_spo2_calc_r();
    m_spo2_calc_spo2();
    if (kalman_filters[corr_filt].kalman_avg > 0)
    { 
      // For beat to beat detection. This takes about 15ms. So if you aren't doing breath detection turn it off. (Longer for slower beats.)
      m_spo2_triangle_filter_new_arr(AC1f, (uint8_t)(INTERVAL + ppg_fir_filter_length), ac_filtered, num_slopes, slopes, lengths, start_val, subtract_avg);
    }
    if (kalman_filters[corr_filt].kalman_avg == 0)
      m_spo2_avg_n_samples(AC1f, ARRAY_LENGTH, samples2avg / 2); // Don't smooth as much in case the peak is fast.
    else
    {
      m_spo2_avg_n_samples(AC1f, ARRAY_LENGTH, samples2avg); // Smooth the data more to avoid detecting peaks from the dichrotic notch
    }
    fast = m_spo2_find_max_corr(AC1f, (uint16_t)SAMPLE_LENGTH / 2, offset_guess);
    if (!fast)
    {
      if (kalman_filters[corr_filt].kalman_avg == 0)
      {
        m_spo2_avg_n_samples(AC1f, ARRAY_LENGTH, samples2avg / 2); // Smooth the data more to avoid detecting peaks from the dichrotic notch
      }
      m_spo2_find_max_corr(AC1f, (uint16_t)SAMPLE_LENGTH, offset_guess); // AC1f is the IR sample after get_rms is run
    }
    if (kalman_filters[corr_filt].kalman_avg > 0)
    {
      m_spo2_peak2peak(ac_filtered); // Returns b2b sample time differences in spo2.b2b1f and also ppg_falltime. ppg amplitude can be calculated as well (instead of RMS)
    }

    m_spo2_calc_hr();
    if (current_hr1f == 0)
    {
      // Don't take valid spo2 samples if we can't find a heart rate
      current_spo21f = 0;
    }

    run_kalman(hr_filt, current_hr1f, NO_RATIO); // This is a traditional 8 second average of the heart rate that produces a medically relevant value.

    // The effective filter length is slightly longer when we apply the correlation filter, which does some pre-averaging. To first order add the filter lengths in quadrature sqrt(8^2+3^3) = 8.5 sec
    run_kalman(spo2_filt, current_spo21f, NO_RATIO); // This is a traditional several second flat average of the heart rate that produces a medically relevant value. 9 seconds matches the reference meter best.
  }
  else
  {
    avg_hr1f   = 0;
    prev_valid = 0;
    avg_spo21f = 0;
  }
  display_spo2 = ((kalman_filters[spo2_filt].kalman_avg >> FIXED_BITS) * 10) + (((0x0008 & kalman_filters[spo2_filt].kalman_avg) == 0) ? 0 : 5);
  display_hr   = kalman_filters[hr_filt].kalman_avg >> FIXED_BITS;

  // Reduce averaging for fast heart rates. Ramp from MAX_FILTER_LENGTH down to MIN_FILTER_LENGTH from max HR to min heart rate
  if (avg_hr1f > 0)
  {
    samples2avg = MAX_FILTER_LENGTH - ((uint32_t)(MAX_FILTER_LENGTH - MIN_FILTER_LENGTH) * ((uint32_t)(avg_hr1f >> FIXED_BITS) - (uint32_t)40)) / (uint32_t)160;
  }
}

void spo2_do_algorithm_part3(void)
{
  if (kalman_filters[corr_filt].kalman_avg > 0)
  {
    // Do b2b detection
    alg_count++; // Increment the algorithm run count. Like a 1 second tick. Keeps track of peak and valley positions.

    if (systole_found != 0)
    {
      hr_data_buffer[hr_data_buffer_ind] = systole; // Log the systole height (single-sided AM method)
      int ind = 0;

      for (int n = 0; n < MAX_BREATH_ARRAY_LENGTH; n++)
      {
        // Newest sample first, oldest sample last
        ind = hr_data_buffer_ind - n;
        if (ind < 0)
        {
          ind += MAX_BREATH_ARRAY_LENGTH;
        }
        lin_buffer[n] = hr_data_buffer[ind];
      }
      hr_data_buffer_ind++;
      if (hr_data_buffer_ind >= MAX_BREATH_ARRAY_LENGTH)
      {
        hr_data_buffer_ind = 0;
        do_rr_alg          = 1;
      }

      if (do_rr_alg)
      {
        for (int m = 0; m < NUM_RR_FILTERS; m++)
        {
          uint8_t   filter_len = rr_filter_lens[m];
          int32_t   temp       = 0;
          rr_filter[0]         = 0;

          // Get the running baseline
          for (int p = 0; p < (int)MAX_BREATH_FILTER_SPAN; p++)
          {
            temp = 0;
            for (int n = 0; n < filter_len; n++)
            {
              temp += lin_buffer[p + n];
            }
            rr_filter[p] = temp / (int32_t)filter_len;
          }
          baseline_sign_changes[m] = m_spo2_count_sign_changes(rr_filter, filter_len);
          for (int n = 0; n < MAX_BREATH_FILTER_SPAN; n++)
          {
            detrended[n] = lin_buffer[n + filter_len / 2] - rr_filter[n];
          }
          zero_crossings[m] = m_spo2_count_zero_crossings(detrended, (uint8_t)MAX_BREATH_FILTER_SPAN);
        }
      }
    }
  }
}

void spo2_do_algorithm_part4(void)
{
  int display_avg = 0;
  int32_t numer;
  int32_t denom;

  if (filter_tracker == 0)
  {
    filter_tracker = 3 * FILTER_DAMPING + 1; // Start by assuming we need center filter (centered in range)
  }

  if (do_rr_alg)
  {
    // NOW DECIDE WHICH FILTER IS RECENTLY PERFORMING THE BEST
    for (int m = 0; m < NUM_RR_FILTERS; m++)
    {
      // Adding fixed precision to RMS
      m_spo2_characterize_array_iir(m, (((int16_t)zero_crossings[m]) << FIXED_BITS), iir_avg, iir_rms, iir_rms_smooth); // Get the RMSE for each filter using a nonlinear IIR estimation (saves RAM and multiplications). This is tuned for an effective RMSE over 10 samples.

      if (baseline_sign_changes[m] > 0)
      {
        // Protect against div by 0
        // Adding more fixed precision to keep resolution after division
        numer      = ((int32_t)iir_rms_smooth[m]) << FIXED_BITS;
        denom      = baseline_sign_changes[m];
        badness[m] = numer / denom;
      }
      else
      {
        badness[m] = 0; // This will make this filter not used
      }
    }

    max_fom = 3; // Default
    for (int n = 0; n < NUM_RR_FILTERS; n++)
    {
      if ((badness[n] > 0) && (badness[n] < badness[max_fom]))
      {
        max_fom = n;
      }
    }

    // Key to stability is damping the switching of filters. Increase FILTER_DAMPING for more stability. Decrease for more response.
    // This is the new slope limiter
    if (current_filter == 0)
    {
      current_filter = filter_tracker / FILTER_DAMPING;
    }
    if (filter_tracker < (max_fom * FILTER_DAMPING + (FILTER_DAMPING >> 1)))
    {
      filter_tracker++; // Increment filter
    }
    else if (filter_tracker > (max_fom * FILTER_DAMPING + (FILTER_DAMPING >> 1)))
    {
      filter_tracker--; // Decrement filter
    }
    current_filter = (filter_tracker / FILTER_DAMPING);

    if (zero_crossings[current_filter] != 0)
    {
      // Skipping zero values
      rr_current_monitor1f = (((uint32_t)MAX_BREATH_FILTER_SPAN * 2) << FIXED_BITS) / zero_crossings[current_filter];
      run_kalman(rr_filt, rr_current_monitor1f, NO_RATIO);
    }

    if (kalman_filters[rr_filt].kalman_avg != 0)
    {
      if (display_avg_cnt < DISPLAY_NUM2AVG)
      {
        display_avg_cnt++; // Ramp average at the beginning (better than initializing to first value)
      }

      display_avg_buffer[display_avg_buffer_ind] = kalman_filters[rr_filt].kalman_avg;
      display_avg_buffer_ind++;
      if (display_avg_buffer_ind == DISPLAY_NUM2AVG)
      {
        display_avg_buffer_ind = 0; // Loop index
      }

      display_avg = 0;              // Initialize
      for (int n = 0; n < display_avg_cnt; n++)
      {
        display_avg += display_avg_buffer[n];
      }
      display_avg /= display_avg_cnt;
    }

    if (display_avg != 0)
    {
      // Below here isn't updated yet
      breathing_rate1f = ((uint32_t)60) << FIXED_BITS;
      breathing_rate1f *= (uint32_t)SAMPLE_RATE;
      breathing_rate1f /= (uint32_t)INTERVAL;
      breathing_rate1f = breathing_rate1f << FIXED_BITS;
      breathing_rate1f /= (uint32_t)display_avg;
    }
    else
    {
      // Haven't got a good value yet
      breathing_rate1f = 0;
    }
  }
}

void spo2_add_sample(uint32_t ir_data, uint32_t r_data)
{
  raw_data[IR_1][data_ptr] = ((int32_t)ir_data - (int32_t)TARGET_COUNTS);
  raw_data[RED][data_ptr]  = ((int32_t)r_data - (int32_t)TARGET_COUNTS);

  data_ptr++;
  data_ptr = (data_ptr >= ARRAY_LENGTH) ? 0 : data_ptr; // Index roll over
}

uint32_t uint_sqrt(uint32_t val)
{
  // Integer sqrt function from http://www.azillionmonkeys.com/qed/sqroot.html
  uint32_t temp, g = 0, b = 0x8000, bshft = 15;
  do
  {
    if (val >= (temp = (((g << 1) + b) << bshft--)))
    {
      g += b;
      val -= temp;
    }
  } while (b >>= 1);
  return g;
}

uint32_t uint64_sqrt(uint64_t val)
{
  // Integer sqrt function from http://www.azillionmonkeys.com/qed/sqroot.html
  uint64_t temp, g = 0, b = 0x80000000, bshft = 31;
  do
  {
    if (val >= (temp = (((g << 1) + b) << bshft--)))
    {
      g += b;
      val -= temp;
    }
  } while (b >>= 1);
  return g;
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Spo2 get idx
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_get_idx(void)
{
  int16_t val;
  for (uint16_t n = 0; n < downsampled_array_length; n++)
  {
    val      = -downsampled_max_centered_index + n;
    idx2[n]  = val * val;
  }
}

/**
 * @brief 
 * 
 * Copies all data from the raw_data buffers to temporary buffer and subtracts the DC level
 * Output is AC1f-->extended precision array
 * This data is oldest first (like viewing captured data on a screen)--matters for shape of b2b FIR filter
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_copy_data(uint8_t channel)
{
  indx = data_ptr;

  for (int n = 0; n < ARRAY_LENGTH; n++)
  {
    AC1f[n] = ((int32_t)raw_data[channel][indx] << FIXED_BITS) - mean1f[channel]; // Load the ~11bit or less AC data into an array with fixed precision for DC removal, etc.
    indx++;
    if (indx == ARRAY_LENGTH)
    {
      indx = 0; // Loop back
    }
  }

  if (channel == 1)
  {
    first_data = (uint32_t)((int32_t)raw_data[channel][indx1] + TARGET_COUNTS);
  }
}

/**
 * @brief 
 * 
 * Calculates the mean DC level being subtracted in mean and residual DC level
 * for each channel and stores is in res_dc. THe mean is used for SpO2 calculations.
 * There is a lag in the mean and rms but this should not be significant as the mean
 * is quite constant and changes in SpO2 are usually due to rms.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_get_dc(void)
{
  for (uint8_t channel = 0; channel < 2; channel++)
  {
    mean1f[channel] = 0; // Get the running mean
    for (uint16_t n = 0; n < ARRAY_LENGTH; n++)
    {
      mean1f[channel] += raw_data[channel][n];
    }
    mean1f[channel] = (mean1f[channel] << FIXED_BITS) / ARRAY_LENGTH;
  }
}


/**
 * @brief 
 * 
 * This takes samples from the IR and Red DC data buffers and calculates the rms and
 * mean values needed for the SpO2 calculation. It reuses the AC1f data array for both
 * Red and IR to same memory.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_get_rms(void)
{ 
  int32_t slope1f     = 0;
  int32_t parabolic4f = 0;
  int32_t ind;
  uint32_t var1f;

  m_spo2_get_dc(); // Calculate DC level

  for (int channel = 0; channel < 2; channel++)
  {
    var1f = 0;
    m_spo2_copy_data(channel);          // Copies data to AC1f[n] array (extended precision) and removes DC
    m_spo2_avg_n_samples(AC1f, ARRAY_LENGTH, 8); // Do the 8 sample avg we used to do before loading

    // Remove slope
    slope1f = 0;
    ind = -(int32_t)downsampled_max_centered_index;

    // Calc slope
    for (uint16_t n = 0; n < downsampled_array_length; n++)
    {
      slope1f += (int32_t)AC1f[ds_start + (1 << DOWNSAMPLE_BITS) * n] * ind;
      ind++;
    }
    slope1f /= sum_squares;             // Normalize
    slope1f /= 1 << DOWNSAMPLE_BITS;    // Reduce slope by downsampling ratio

    // Remove linear slope
    ind = -(int16_t)((ARRAY_LENGTH - 1) >> 1);
    for (uint16_t n = 0; n < ARRAY_LENGTH; n++)
    {
      AC1f[n] -= ind * slope1f;
      ind++;
    }

    // Calc quadratic term
    ind = 0;
    for (uint16_t n = 0; n < downsampled_array_length; n++)
    {
      parabolic4f += (int32_t)idx2[n] * (int32_t)AC1f[ds_start + (1 << DOWNSAMPLE_BITS) * n];
      ind++;
    }
    parabolic4f = (int32_t)((((int64_t)parabolic4f) << (FIXED_BITS * 3)) / (int64_t)sum_quads); // Use 12 more bits fixed precision and int64s for the intermediate calculation
    parabolic4f /= (1 << DOWNSAMPLE_BITS) * (1 << DOWNSAMPLE_BITS);

    // Remove quadratic term
    ind = -(int16_t)((ARRAY_LENGTH - 1) >> 1);
    for (uint16_t n = 0; n < ARRAY_LENGTH; n++)
    {
      AC1f[n] -= (int32_t)(((int64_t)((int32_t)ind * (int32_t)ind) * (int64_t)parabolic4f) >> (FIXED_BITS * 3));
      ind++;
    }

    // Calculate residual dc after parabolic removal
    int32_t res_sum = 0;
    for (uint16_t n = 0; n < ARRAY_LENGTH; n++)
    {
      res_sum += AC1f[n];
    }
    res_sum /= ARRAY_LENGTH;

    // Remove residual dc after parabolic removal
    ind = -(int16_t)((ARRAY_LENGTH - 1) >> 1);
    for (uint16_t n = 0; n < ARRAY_LENGTH; n++)
    {
      AC1f[n] -= res_sum;
    }

    // Calculate variance
    for (uint16_t n = MAX_FILTER_LENGTH; n < SAMPLE_LENGTH + MAX_FILTER_LENGTH; n++)
    {
      var1f += ((uint32_t)labs(m_spo2_div2n32(AC1f[n], 2))) * ((uint32_t)labs(m_spo2_div2n32(AC1f[n], 2))); // Getting 2 bit shifts here, taking out extra bit shift for overhead
    }

    rms1f[channel] = uint_sqrt(var1f / (uint32_t)SAMPLE_LENGTH);                                                                  // Square root halfs the bit shifts back to 2, so this is more like RMS0.5f -- OH WELL (it is 4x bigger, not 16x bigger)
                                                                                                                                  // Perfusion_index[channel] = (float)(rms1f[channel]*141*PI_CALIBRATION) / (float)( (mean1f[channel]>>FIXED_BITS)+(int32_t)TARGET_COUNTS); // x100 for percent and xsqrt(2) to change from RMS to approximately P2P, also scaling PI by 4 to make it comparable to transmission PPG measurements (rough calibration)
    perfusion_index[channel] = (float)(rms1f[channel] * 141) / (float)((mean1f[channel] >> FIXED_BITS) + (int32_t)TARGET_COUNTS); // X100 for percent and xsqrt(2) to change from RMS to approximately P2P, also scaling PI by 4 to make it comparable to t
  }
}

/**
 * @brief 
 * 
 * An error of 0.01 in R is an error of about 0.25% in SpO2. So we want to keep
 * at least 8 bits of precision in R
 * for a huge RMS value like 2056 for IR and 1024 for R.
 * IR: (11 bits+4bits fp=15bits) we  shift 17 bits to 32 bits
 * R: (10 bits+4bits fp=14bits) we shift 17 bits to 31 bits
 * then we divide by the mean ~2e5, (18bits) which leaves 14 bits for IR and 13 for red.
 * We can then shift the red that by 12 bits before we divide by the denominator all 13 bits.
 * Now consider a minimum rms of 32 (5 bits) for R and 64 bits for IR
 * IR: 6bit+4bits FP=10bits. We shift by 17 bits to 27 bits
 * R: 5bit+4bits FP=9bits. We shift by 17 bits to 26 bits
 * then we divide by 18 bits which leaves 8 bits. This is bare bones
 * a 3 fixed point (nibble) shifts
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_calc_r(void)
{
  R = ((float)rms1f[RED] / (float)((mean1f[RED] >> FIXED_BITS) + (int32_t)TARGET_COUNTS)) /
      ((float)rms1f[IR_1] / (float)((mean1f[IR_1] >> FIXED_BITS) + (int32_t)TARGET_COUNTS)); // 3ms
}

/**
 * @brief         Spo2 calculate spo2
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_calc_spo2(void)
{
  float spo2 = 0;

  // Trying to be very efficient with floating point multiplication here
  float Rs[5];
  float c[6] = {0, 0, 0, -28.149, -10.293, 107.36};
  float Rsq  = R * R;
  float R4th = Rsq * Rsq;

  Rs[0] = R4th * R;  // R^5
  Rs[1] = R4th;      // R^4
  Rs[2] = Rsq * R;   // R^3
  Rs[3] = Rsq;       // R^2
  Rs[4] = R;         // R
  spo2  = c[5];      // Constant term

  for (uint8_t n = 0; n < 5; n++)
  {
    spo2 += c[n] * Rs[n];
  }

  current_spo21f = (int16_t)(spo2 * (1 << FIXED_BITS));
  current_spo21f = ((current_spo21f >> FIXED_BITS) > 100) ? 100 << FIXED_BITS : current_spo21f;
  current_spo21f = ((current_spo21f >> FIXED_BITS) < MIN_SPO2) ? MIN_SPO2 << FIXED_BITS : current_spo21f;
}

/**
 * @brief 
 * 
 * Calculates heart rate from a beat periodicity (samples per period aka "offset" in fixed precision
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_calc_hr(void)
{
  // This is a short duration filter designed to snag bad correlation function inputs and replace them with reasonably close values in time.
  run_kalman(corr_filt, final_offset1f, NO_RATIO);
  if (kalman_filters[corr_filt].kalman_avg == 0)
  {
    current_hr1f = 0;
  }
  else
  {
    current_hr1f = ((uint32_t)((uint32_t)SAMPLE_RATE_PER_MIN << FIXED_BITS) << FIXED_BITS) / (uint32_t)kalman_filters[corr_filt].kalman_avg;
  }
}

/**
 * @brief 
 * 
 * This does a correlation of the data with time-offset data and returns the dot
 * product of the two arrays. It takes arrays of ACdata with fixed precision.
 * I modified it to start after max filter length since the lag is linearly increasing
 * in the region and it can cause some error in the HR estimate.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static int32_t m_spo2_corr(int16_t *x, uint16_t len, uint16_t offset)
{
  int32_t result = 0;
  offset = (offset > SAMPLE_LENGTH) ? SAMPLE_LENGTH : offset;  // Conform inputs to limits
  len    = (len > SAMPLE_LENGTH) ? SAMPLE_LENGTH : len;

  for (uint16_t n = MAX_FILTER_LENGTH; n < (len + MAX_FILTER_LENGTH); n++)
  {
    // Remove 4 bits of precision to keep same precision, remove 4 more for overhead.
    // Cast as int32_t for the multiplication to avoid overflow,
    result += (((int32_t)x[n]) >> FIXED_BITS) * (((int32_t)x[n + offset]) >> 4); 
  }
  return result;
}

/**
 * @brief 
 * 
 * This program looks for the peak of the autocorrelation function by doing correlations and stepping
 * down then up in offset until it finds a peak. If it exits at the extrema of the range
 * then this is considered a fail. That doesn't quite work well with the variable stepping
 * at long hear rates, however the kalman can sometimes throw out a bad low heart rate
 * Fine search for correlation peak using defined step size (index units).
 * Finds peak and interpolates the maximum and saves the answer with fixed precision.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_fine_search(int16_t *x, uint16_t len, uint16_t start_offset, int32_t start_correl, uint16_t max_search_step)
{
  uint32_t search_step = start_offset / 30; // Dynamically stepping to get better resolution at high heart rates
  search_step = (search_step > max_search_step) ? max_search_step : search_step;

  uint16_t offset = start_offset;
  int32_t c;
  int32_t high_side;
  int32_t low_side;
  int32_t lowest;
  final_correl = start_correl;
  final_offset = start_offset;
  uint16_t elapsed_time = bsp_time_now() - p2_start_time;
  int32_t y[3];

  if (elapsed_time >= 15)
  {
    final_correl   = 0;
    final_offset   = 0;
    final_offset1f = 0;
  }
  else
  {
    while (1)
    {
      // Search toward lower offset, higher frequency
      offset -= search_step;
      if (offset >= min_offset)
      {
        c = m_spo2_corr(x, len, offset);
        if (c <= final_correl)
        {
          break;                    // Getting worse. Stop.
        }
        else
        {
          high_side = final_correl; // Load prev sample as new larger offset sample for fit
          final_correl = c;         // Better, use new result; keep going
          final_offset = offset;
        }
      }
      else
      {
        c = start_correl;
        break;
      }
    }
    low_side = c; // The low-side sample is the one we exited the loop on

    if (final_correl == start_correl)
    {
      // Didn't find something bigger at lower offsets, check higher
      offset = start_offset;
      while (1)
      {
        offset += search_step;
        if (offset < max_offset)
        {
          c = m_spo2_corr(x, len, offset); // Search lower frequency
          if (c <= final_correl)
          {
            break;                         // Getting worse; stop
          }
          else
          {
            low_side     = final_correl;
            final_correl = c;               // Better, use new results; keep going
            final_offset = offset;
          }
        }
        else
        {
          break;                            // Out of range; stop
        }
      }
      high_side = c;                        // The high side sample is the one we exited the loop on.
    }
    if (final_offset <= min_offset)
    {
      final_offset1f = min_offset << FIXED_BITS; // Force the algorithm to bonk at max found check.
    }
    else if (final_offset >= max_offset)
    {
      final_offset1f = max_offset << FIXED_BITS; // Force the algorithm to bonk at max found check.
    }
    else
    {
      // Only run this if final_offset is not one of the boundaries
      lowest = (high_side < low_side) ? high_side : low_side;
      if (final_correl - lowest == 0)
      {
        final_offset1f = final_offset << FIXED_BITS;
      }
      else
      {
        y[0] = low_side;
        y[1] = final_correl;
        y[2] = high_side;
        m_spo2_simple_peak_find(y, &final_offset1f, &fit_correl, (uint16_t)final_offset, (int32_t)search_step);
      }
    }
  }
}

/**
 * @brief         Check for a max in the correlation in a region
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static uint8_t m_spo2_check_4_max(int16_t *x, uint16_t len, uint16_t start_offset, int32_t start_correl)
{
  uint8_t max_found = 0;
  m_spo2_fine_search(x, len, start_offset, start_correl, MAX_FINE_STEP);
  if (final_offset1f != (min_offset << FIXED_BITS))
  {
    if (final_offset1f != (max_offset << FIXED_BITS))
    {
      max_found = 1;
    }
  }
  return max_found;
}

/**
 * @brief 
 * 
 * This is the brains of the heart rate detection. It looks for the minimum of the
 * autocorrelation using large steps, interpolates the minimum, doubles it and does a fine
 * search near the peak. It is quite robust.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static uint8_t m_spo2_find_max_corr(int16_t *x, uint16_t max_length, uint16_t offset_guess)
{
  uint8_t  rising     = 0;
  uint8_t  fail       = 0;
  uint16_t try_offset = MIN_OFFSET - BIG_STEP;
  uint16_t offsets[3];
  offsets[0] = try_offset;
  int16_t offset_extremum1f;
  int32_t c[3];
  int32_t prev_correl;
  int32_t min_correl;
  int32_t start_correl;
  uint8_t max_found;

  if (prev_valid)
  {
    prev_offset = final_offset;
    prev_correl = final_correl;
  }

  uint16_t samples2use    = max_length;
  c[0]                    = m_spo2_corr(x, samples2use, try_offset);
  try_offset              += BIG_STEP;
  offsets [1]             = try_offset;
  c[1]                    = m_spo2_corr(x, samples2use, try_offset);
  rising                  = (c[1] > c[0]) ? 1 : 0;                    // Skip to search for a peak if rising, else look for a minimum and double it.
  uint16_t step           = BIG_STEP;                                 // Start with big step
  const uint8_t max_step  = 18;

  if (!rising)
  {
    while (!rising)
    {                                    // Keep going until you find a minimum
      step += (step < max_step) ? 1 : 0; // Increment step size if less than max step
      uint16_t elapsed_time = bsp_time_now() - p2_start_time;

      try_offset += step; // Increment by step size
      if (try_offset > MAX_OFFSET || (elapsed_time >= 15))
      {
        fail = 1; // Still falling and ran out of samples
        break;
      }

      c[2] = m_spo2_corr(x, samples2use, try_offset);
      if (c[2] > c[1])
      {
        rising = 1;
      }
      else
      {
        c[0]       = c[1];
        c[1]       = c[2];
        offsets[0] = offsets[1];
        offsets[1] = try_offset;
      }
    }
    if (!fail)
    {
      offsets[2] = try_offset;
      m_spo2_peak_find(offsets, c, &offset_extremum1f, &min_correl);
      offset_guess = (uint16_t)(offset_extremum1f >> (FIXED_BITS - 1));
    }
  }
  else
  {
    // Already rising
    while (rising)
    {
      // Keep going until you find a drop
      uint16_t elapsed_time = bsp_time_now() - p2_start_time;
      try_offset += step;
      step += (step < max_step) ? 1 : 0; // Increment step size if less than max step
      if ((try_offset > MAX_OFFSET) || ((elapsed_time >= 15)))
      {
        fail = 2; // Still rising and ran out of samples
        break;
      }
      else
      {
        c[2] = m_spo2_corr(x, samples2use, try_offset);
        if (c[2] < c[1])
        {
          rising = 0;
        }
        else
        {
          c[0]       = c[1];
          c[1]       = c[2];
          offsets[0] = offsets[1];
          offsets[1] = try_offset;
        }
      }
    }
    if (!fail)
    {
      offsets[2] = try_offset;
      m_spo2_peak_find(offsets, c, &offset_extremum1f, &min_correl);
      offset_guess = (uint16_t)(offset_extremum1f >> FIXED_BITS);
    }
    else
    {
      offset_guess = DEFAULT_GUESS;
    }
  }

  // Condition inputs
  if (offset_guess < MIN_OFFSET)
    offset_guess = DEFAULT_GUESS;
  if (offset_guess > MAX_OFFSET)
    offset_guess = DEFAULT_GUESS;

  start_correl = m_spo2_corr(x, samples2use, offset_guess);
  max_found    = m_spo2_check_4_max(x, samples2use, offset_guess, start_correl);

  if (prev_valid & !max_found)
  {
    // Go ahead and try the previous value if the other bonked.
    // Note you can't just rely on this or you could lock into a harmonic.
    max_found = m_spo2_check_4_max(x, samples2use, prev_offset, prev_correl);
  }

  if (!max_found)
  {
    final_correl   = 0;
    final_offset1f = 0;
    prev_valid     = 0;
  }
  else
  {
    prev_valid = 1;
  }
  return max_found;
}

/**
 * @brief 
 * 
 *  This is a peak (or valley) finder that takes arbitrary x and y data. The y
 *  data here is not kept in fixed precision so it assumes it is dealing with large
 *  numbers like the correlation dot products. It is more general than the simple finder
 *  which assumes equal spacing in the x data (offset steps)
 *  Solve this linear equation
 *  a*x[0]^2 + b*x[0] + c = y[0]
 *  a*x[1]^2 + b*x[1] + c = y[1]
 *  a*x[2]^2 + b*x[2] + c = y[2]
 * 
 *  Using x2 = {x[0]^2, x[1]^2, x[2]^2}
 *  Subtract x[1] from x's to get centered values, y[1] = C, reduces to system of 2 equations in for constants y_i-C
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_peak_find(uint16_t *x0, int32_t *y, int16_t *x_fit1f, int32_t *y_fit)
{
  int32_t x[2]  = {(int32_t)x0[0] - (int32_t)x0[1], (int32_t)x0[2] - (int32_t)x0[1]};  // Get centered x data
  int32_t x2[2] = {x[0] * x[0], x[1] * x[1]};                                          // Get squared x data
  int32_t C     = y[1];                                                                // Second equation in system is trivial
  int32_t yp[2] = {y[0] - C, y[2] - C};                                                // Get centered y values
  int32_t D;
  int32_t Dx;
  int32_t Dy;
  int32_t A;
  int32_t B;

  // Using Cramer's rule
  D  = x2[0] * x[1] - x[0] * x2[1];    // Get determinant
  Dx = yp[0] * x[1] - x[0] * yp[1];    // get det with constants (y) replacing col 1
  Dy = x2[0] * yp[1] - yp[0] * x2[1];  // Get det with constants (y) replacing col 2
  A  = Dx / D;                         // Not using fixed precision because y is so large
  B  = Dy / D;

  // Peak is where slope = 0, namely dy/dx = 2ax+b = 0, so x_peak = -b/2a and add back in offset x0[1]
  // Plug x = -b/2a into y = ax^2 + bx +c
  // If a > 0 it is a peak
  // If a < 0 it is a valley
  *x_fit1f = (int16_t) - (B << FIXED_BITS) / (2 * A) + (x0[1] << FIXED_BITS);
  *y_fit   = C - (((B / 4) * B) / A);
}

/**
 * @brief 
 * 
 * This is a fast peak finder for equally spaced data in x. Y is not calculated with
 * additional fixed precision so the estimated peak value is accurate for large values of y only.
 * For numerical stability let x[1] = 0, namely the middle point (high or low) is zero
 * To simplify the math we measure in units of the step size, so xfit1f is fixed precision in units of the step size
 *
 * Solve this linear equation
 * a*x[0]^2 + b*x[0] + c = y[0]
 * a*x[1]^2 + b*x[1] + c = y[1]
 * a*x[2]^2 + b*x[2] + c = y[2]
 *
 * Using x =[-1,0,1}, and x2 = {x[0]^2, x[1]^2, x[2]^2} = {1,0,1]
 * Solve for a, b, and c using x2, x and c as constants
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_simple_peak_find(int32_t *y, int16_t *x_fit1f, int32_t *y_fit, uint16_t x_center, int16_t step)
{
  int32_t A; // A
  int32_t B; // B
  int32_t C; // C

  // Using Cramer's rule
  C = y[1];                        // Middle equation in system is trivial. Problem reduces to 2x2 determinant.
  A = (y[0] + y[2] - (C * 2)) / 2; // Dx/D
  B = (y[2] - y[0]) / 2;           // Dy/D

  *x_fit1f = (int16_t)(-((B << FIXED_BITS) / A) * (int32_t)step) / 2 + (((int32_t)x_center) << FIXED_BITS);  // -b/2a * step + x_center
  *y_fit   = C - (((m_spo2_div2n32(B, 2)) * B) / A);                                                         // C - b^2/4a
}

/**
 * @brief 
 * 
 * Runs a moving average filter over the data in the array.
 * The filter length ramps up to the desired length so the first few samples
 * don't have the same lag as latter ones. For this reason the rms calc starts after 
 * the max sample length
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_avg_n_samples(int16_t *x, int16_t len, uint8_t number2avg)
{
  int32_t avg_buffer[MAX_FILTER_LENGTH];
  uint16_t buffer_ind  = 0;
  uint8_t  avg_length  = 0;
  int32_t  running_sum = 0;
  for (uint16_t n = 0; n < len; n++)
  {
    if (avg_length < number2avg)
    {
      avg_length++;
    }
    else
    {
      running_sum -= avg_buffer[buffer_ind];
    }

    avg_buffer[buffer_ind] = (int32_t)x[n];
    running_sum            += avg_buffer[buffer_ind];
    x[n]                   = (int16_t)(running_sum / avg_length);
    buffer_ind++;

    if (buffer_ind == number2avg)
      buffer_ind = 0;
  }
}

/**
 * @brief         Spo2 get direction
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static int m_spo2_get_direction(int32_t data1, int32_t data2)
{
  int dir;
  if (data2 > data1)
  {
    dir = 1;
  }
  else if (data2 < data1)
  {
    dir = -1;
  }
  else
  {
    dir = 0;
  }
  return dir;
}

/**
 * @brief         Spo2 find min max 32
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_findminmax32(int32_t *data, uint16_t start_ind, uint16_t stop_ind, int32_t *c, int *extreme, int *type)
{
  uint16_t ind = start_ind;
  c[1] = data[ind];
  ind = ind + 1;
  c[2] = data[ind];
  int prev_dir = 0; // Direction prior to getting a zero direction
  uint8_t change_dir = 0;
  int dir1;
  int dir2;
  dir2 = m_spo2_get_direction(c[1], c[2]); // -1 is falling, 1 is rising
  ind = ind + 1;
  *extreme = -1; // Default = fail condition
  *type = 0;

  while (!change_dir && (ind <= stop_ind))
  {
    c[0] = c[1];
    c[1] = c[2];
    c[2] = data[ind];
    dir1 = dir2;      // Slide over previous value.
    dir2 = m_spo2_get_direction(c[1], c[2]);

    if (dir2 * dir1 < 0)
    {
      // Changed direction
      change_dir = 1;
      *type      = dir1;  // 1 = peak, -1 = valley
      break;
    }
    else if ((dir1 == 0) && (prev_dir * dir2 < 0))
    {
      // Changed direction after a flat
      change_dir = 1;
      *type      = prev_dir;
      break;
    }
    else if ((dir2 == 0) && (dir1 != 0))
    {
      // Log the direction we were going before we reached a flat
      prev_dir = dir1;
    }
    ind = ind + 1;
  }
  *extreme = ind - 1; // This is the last point we verified is or is not an extremum. Start next search at this point.
  return;
}

/**
 * @brief 
 * 
 * Beat to beat period detection, amplitude detection, and ppg fall time
 * Most recent b2b amplitude stored asSPO2 class variable b2b.
 * Import from xAC one interval of data, to avoid overcounting. Even so, we could occasionally overcount when the filter length changes and the lag shifts. So we put in checks for repeated valleys or peaks.
 * Missed peaks or valleys will hopefully be caught by the Kalman filter.
 * Note, for decent performance, xAC must be filtered with an asymmetric triangle filter: 3/4 of time rising (shallow), and 1/4 of time falling (steep), to be a matched filter for PPG detection/
 * the filter length must be chosen by the correlation-based estimated of the heart beat period. This is computationally expensive.
 * Downsampling is not a good idea. Matlab sims show it does not have good enough HR accuracy.
 * Note that we are comparing samples from different runs of the algorithm with different DC offsets, slope and 2nd order regression. The amplitude is not necessarily as precise as if we had run a continuous bandpass filter through the data.
 * An adaptive bandpass filter was not implemented in this code so this the best we can do.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_peak2peak(int32_t *ac_data)
{
  static uint8_t extreme_cnt = 0;
  int extreme;
  int     type                    = 0;
  int32_t step                    = 1;
  static  int32_t prev_peak_x1f   = 0;
  static  int32_t prev_peak_y     = 0;
  static  int16_t prev_type       = 0;
  static  uint8_t valley_detected = 0;

  int32_t y[3];
  int16_t x1f;  // Latest peak position
  int32_t yfit; // Latest peak height
  int32_t c[3];
  uint8_t use_corr  = 0;
  uint8_t array_ind = 0;
  uint8_t array_end = INTERVAL; // Extreme on exit (no peak) will be at interval (interval+1 samples) and next search starts here. We search INTERVAL+1 samples and overlap 1 sample.
  int16_t trial_time;

  if (kalman_filters[corr_filt].kalman_avg > 0)
  {
    use_corr = 2;
  }
  else if (final_offset1f > 0)
  {
    use_corr = 1;
  }
  while (1)
  {
    // Look for extrema
    m_spo2_findminmax32(ac_data, array_ind, array_end, c, &extreme, &type);
    array_ind = extreme; // Update start position for next time;

    if (type != 0)
    {
      // Extreme (peak or valley) found
      y[0] = ac_data[extreme - 1];
      y[1] = ac_data[extreme];
      y[2] = ac_data[extreme + 1];
      extreme_cnt++;

      if (extreme_cnt > 0)
      {
        if ((type == -1) && (prev_type == 1))
        {
          // Get ppg fall time and optionally peak amplitude
          m_spo2_simple_peak_find(y, &x1f, &yfit, (uint16_t)extreme, step);
          systole       = yfit;
          systole_found = 1;                                                                                 // Starts collecting data for RR algorithm
          peak_amp      = prev_peak_y - yfit;                                                                // Optional to track b2b PI or for b2b-based SpO2 (requires recalibration of r-curve)
          trial_time    = (int16_t)((int32_t)x1f + (alg_start_sample_count << FIXED_BITS) - prev_peak_x1f);

          if (use_corr == 2)
          {
            if (trial_time < (final_offset1f >> 3))
            {
              // Our PPG falltime can't be less than 1/8 of the current periodicity estimate.
              trial_time = 0;
            }
          }
          else if (use_corr == 1)
          {
            if (trial_time < (kalman_filters[corr_filt].kalman_avg >> 3))
            {
              trial_time = 0;
            }
          }
          if (trial_time > 0)
          {
            ppg_falltime = trial_time;  // Note it is possible to do some correction for varied lag due to changing filter length, but I will ignore this. FYI, for this filter and waveform shape, lag of peaks is filter_length/4+1, Lag of valleys is filter_length/2+1.
          }                             // If we didn't get a long enough PPG falltime, we simply won't update the previous value. This is contingent on a valid final_offset1f reading, so OR this with corr_filt kalman avg
          valley_detected = 1;
        }
        else if (type == 1)
        {
          // Found a peak
          m_spo2_simple_peak_find(y, &x1f, &yfit, (uint16_t)extreme, step);
          if ((prev_type == -1) && valley_detected)
          {
            // Found a beat
            trial_time = (int16_t)((int32_t)x1f + (alg_start_sample_count << FIXED_BITS) - prev_peak_x1f);
            if (use_corr == 2)
            {
              if (trial_time < (final_offset1f >> 1))
              {
                // Our B2B time can't be less than 1/2 of the current periodicity estimate.
                trial_time = 0;
              }
            }
            else if (use_corr == 1)
            {
              if (trial_time < (kalman_filters[corr_filt].kalman_avg >> 1))
              {
                // Our b2b time can't be less than 1/8 of the current periodicity estimate.
                trial_time = 0;
              }
            }
            if (trial_time > 0)
            {
              b2b1f = trial_time; // Note it is possible to do some correction for varied lag due to changing filter length, but I will ignore this. FYI, for this filter and waveform shape, lag of peaks is filter_length/4+1, Lag of valleys is filter_length/2+1.
            }
          }
          prev_peak_x1f = (int32_t)x1f + (alg_start_sample_count << FIXED_BITS);
          prev_peak_y   = yfit;
        }
      }
      prev_type = type;
    }
    else
    { 
      // Ran out of points
      break; // Exit loop
    }
  }
}

/**
 * @brief 
 * 
 * This function does an efficient FIR filter using incrementing instead of multiplication to update the filter value
 * Provide a number of slope segments (e.g. 2), their slopes, and segment lengths and specify whether you want the filter average to be subtracted after it is constructed..
 * The trick to this computationally efficient function is that the slopes will be treated as the nearest base 2 exponential, namely 0, +/-1, +/-2, +/-4, etc.
 * so that we can use bit shifted and adding instead of multiplying.
 * A length of 1 for the first segment means that start_val is used but the slope is ignored, to ensure sum(lengths) = filt_len.
 * For stack size calculations num_slopes and filter length are limited .
 *
 * NOTE: This is not constraining the inputs. Update constraints so arr_len is not too long, etc. and need arbitration with bad lengths values
 * Works if you be nice, but if you input bad values in it will crash.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static void m_spo2_triangle_filter_new_arr(int16_t *arr, uint8_t arr_len, int32_t *new_arr, uint8_t num_slopes, int16_t *slopes, uint8_t *lengths, int16_t start_val, uint8_t subtract_avg)
{
#define MAX_FILT_LEN 32
#define MAX_NUM_SLOPES 5

  int      filter_avg = 0;
  uint16_t filt_len   = 0;

  for (int n = 0; n < num_slopes; n++)
  {
    filt_len += lengths[n];
  }

  int peaks[MAX_NUM_SLOPES]; // Points of discontinuities
  peaks[0] = lengths[0];
  for (int n = 1; n < num_slopes; n++)
  {
    peaks[n] = peaks[n - 1] + lengths[n];
  }

  // Get filter slopes as a base 2 logarithm
  int slope_signs[MAX_NUM_SLOPES];
  for (int n = 0; n < num_slopes; n++)
  {
    if (slopes[n] < 0)
    {
      slope_signs[n] = -1;
    }
    else if (slopes[n] == 0)
    {
      slope_signs[n] = 0;
    }
    else
    {
      slope_signs[n] = 1;
    }
    int log_slope = -1; // -1 is used temporarily to keep track of zero slope since its log is infinite.
    if (slopes[n] != 0)
    {
      slopes[n] = abs(slopes[n]); // Get rid of signs so we can take the log.
      while (slopes[n] > 0)
      {
        slopes[n] = slopes[n] >> 1;
        log_slope++;
      }
    }
    if (log_slope == -1)
    {
      // Now replace the user value with the log
      slopes[n] = 0;
    }
    else
    {
      slopes[n] = log_slope;
    }
  }

  // Construct multi-segment triangle filter
  filter[0] = start_val; // Starting value, e.g. 1;
  int segment = 0;
  for (int n = 1; n < filt_len; n++)
  {
    while (n >= peaks[segment])
    {
      // When n reaches a peak we switch to new segment
      segment++;
    }
    if (slope_signs[segment] == -1)
    {
      filter[n] = filter[n - 1] - ((int16_t)1 << slopes[segment]);
    }
    else if (slope_signs[segment] == 0)
      filter[n] = filter[n - 1];
    else
    {
      filter[n] = filter[n - 1] + ((int16_t)1 << slopes[segment]);
    }
  }

  if (subtract_avg)
  {
    // Calc and subtract average to center filter to match +/- swinging PPG signal.
    for (int n = 0; n < filt_len; n++)
    {
      filter_avg += filter[n];
    }

    if (filt_len == 4)
    {
      // Avoid division if possible--it takes a long time
      filter_avg = filter_avg >> 2;
    }
    else if (filt_len == 8)
    {
      filter_avg = filter_avg >> 3;
    }
    else if (filt_len == 16)
    {
      filter_avg = filter_avg >> 4;
    }
    else if (filt_len == 32)
      filter_avg = filter_avg >> 5;
    else
    {
      filter_avg /= (int16_t)filt_len; // This is integer division so some precision may be lost
    }

    for (int n = 0; n < filt_len; n++)
    {
      filter[n] -= filter_avg;
    }
  }

  // Calculate the initial FIR filter * array values, then only increment the values for the next filter iteration instead of re-multiplying every term.
  int64_t running_sum = 0;
  for (int n = 0; n < filt_len; n++)
  {
    running_sum += (int32_t)arr[n] * (int32_t)filter[n];
  }

  int32_t temp_val;
  temp_val = arr[0];
  new_arr[0] = running_sum;

  // Calculate all remaining values in the array by incrementing to the new values based on known filter slope rather than multiplying from scratch
  for (int n = 1; n < arr_len - filt_len + 1; n++)
  {
    running_sum -= (int32_t)filter[0] * temp_val; // Dump the previous first term
    segment = 0;
    for (int m = 0; m < filt_len - 1; m++)
    {
      while ((m + 1) >= peaks[segment])
      {
        segment++;
      }
      if (slope_signs[segment] == 1)
      {
        running_sum -= ((int32_t)arr[m + n]) << slopes[segment]; // Decrement when the slope is positive to get to the new lower value
      }
      else if (slope_signs[segment] == -1)
      {
        running_sum += ((int32_t)arr[n + m]) << slopes[segment]; // Increment when the slope is negative to get to the new higher value
      }
      // No change needed for slope = 0;
    }
    running_sum += (int32_t)filter[filt_len - 1] * (int32_t)arr[n + filt_len - 1]; // Get the newest sample
    temp_val = arr[n];                                                             // The not-yet-updated_term

    new_arr[n] = running_sum;
  }
}

/**
 * @brief 
 * 
 * Max arr_num is 8 the way I am initializing
 * arr_num is the number of the array to update. arr_num is in the range 0:NUM_RR_FILTERS-1
 * NOTE: THIS FUNCTION IGNORES ZEROS IN THE COMPUTATION OF THE AVERAGE--THESE ARE ALGORITHM FAILURES. FOR GENERAL USAGE REMOVE THE "if (new_val !=0)" condition.
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_spo2_characterize_array_iir(uint8_t arr_num, int16_t new_val, int16_t *iir_avg, int16_t *iir_rms, int16_t *iir_rms_smooth)
{
  int16_t kinv = 1;                // 1 == bitshift of 1 or factor of 0.5. In this case where kinv = 0.5, then 1-kinv = 0.5--convenient to bitshift both numbers by one instead of div by 2
  int16_t rms_smooth_len_bits = 1; // This is number of bitshifts, e.g. 1 = divide by a factor of 2
  int16_t avg_len_bits = 1;        // Ditto

  // Initialize rms arrays first value to zero
  if (char_array_iir_initialized == 0)
  {
    for (int n = 0; n < NUM_RR_FILTERS; n++)
    {
      iir_rms[n]        = 0;
      iir_rms_smooth[n] = 0;
    }
  }
  if ((char_array_iir_initialized & (1 << arr_num)) == 0)
  {
    iir_avg[arr_num] = new_val; // Initialize avg array to first value (using bits of char_array_iir_initialized to track which filter nums are char_array_iir_initialized)
    char_array_iir_initialized |= 1 << arr_num;
  }

  // Update IIR filters
  iir_rms[arr_num] = (int16_t)uint_sqrt(((((uint32_t)new_val - iir_avg[arr_num]) * ((uint32_t)new_val - iir_avg[arr_num])) >> kinv) + (((uint32_t)iir_rms_smooth[arr_num] * (uint32_t)iir_rms_smooth[arr_num]) >> kinv)); // sqrt(k*(new_val-avg)^2 + (1-k)*(smooth)^2)
  iir_rms_smooth[arr_num] += m_spo2_div2n((iir_rms[arr_num] - iir_rms_smooth[arr_num]), rms_smooth_len_bits);

  if (new_val != 0)
  {
    iir_avg[arr_num] += m_spo2_div2n((new_val - iir_avg[arr_num]), avg_len_bits);
  }
}

/**
 * @brief         Spo2 div2n
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static int16_t m_spo2_div2n(int16_t num, uint8_t bits)
{
  // Divide by n that is numerically equivalent for negative and positive numbers
  return (num & 0x8000) ? ~((((~num) + 1) >> bits) - 1) : num >> bits;
}
/**
 * @brief         Spo2 div2n32
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

static int32_t m_spo2_div2n32(int32_t num, uint8_t bits)
{
  // Bit shifting divide by 2^n bits that is numerically equivalent for negative and positive numbers (-1 >>1 ) = 0, or -9>>1 = -4 (instead of rounded toward negative infinity (-5) like usual)
  return (num & 0x80000000) ? ~((((~num) + 1) >> bits) - 1) : num >> bits;
}

/**
 * @brief         Spo2 count sign changes
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static uint8_t m_spo2_count_sign_changes(int32_t *arr, uint8_t len)
{
  uint8_t num_sign_changes = 0;
  int     dir              = 0;
  int     new_dir          = 0;

  for (int n = 1; n < len; n++)
  {
    if (arr[n] > arr[n - 1])
    {
      new_dir = 1;
    }
    else if (arr[n] < arr[n - 1])
    {
      new_dir = -1;
    }
    if (new_dir != 0)
    {
      // Going up or down
      if (dir != 0)
      {
        // Not comparing to flat
        if (new_dir != dir)
        {
          // Dir change
          num_sign_changes++;
        }
      }
    }
    dir = new_dir;
  }
  return num_sign_changes;
}

/**
 * @brief         Spo2 count zero crossings
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static uint8_t m_spo2_count_zero_crossings(int32_t *arr, uint8_t len)
{
  uint8_t num       = 0;
  int     prev_sign = 0;

  // Use prev_sign to track the previous nonzero sign to avoid double counting if a value is actually zero.
  if (m_spo2_sign(arr[0]) != 0)
  {
    prev_sign = m_spo2_sign(arr[0]);
  }
  for (int n = 1; n < len; n++)
  {
    if ((prev_sign != 0) && (m_spo2_sign(arr[n]) != prev_sign))
    {
      num++;
    }
    if (m_spo2_sign(arr[n]) != 0)
    {
      prev_sign = m_spo2_sign(arr[n]);
    }
  }
  return num;
}

/**
 * @brief         Spo2 sign
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static int m_spo2_sign(int32_t val)
{
  if (val < 0)
    return -1;
  if (val > 0)
    return 1;
  return 0;
}

/* End of file -------------------------------------------------------- */
