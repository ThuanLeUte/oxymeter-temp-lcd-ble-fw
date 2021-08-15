/**
 * @file       spo2.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-25
 * @author     Thuan Le
 * @brief      Handle calculate heart rate and blood oxigen
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SPO2_H
#define __SPO2_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
#define INTERVAL                      (40)
#define SAMPLE_LENGTH                 (148)   // 40.5 BPM at 100sps, also MAX_OFFSET
#define MAX_OFFSET                    (SAMPLE_LENGTH)
#define MIN_OFFSET                    (30)    // 200 BPM at 100sps
#define MAX_FILTER_LENGTH             (42)    // For avgNsamples filter --MUST BE EVEN
#define MIN_FILTER_LENGTH             (14)    // ~<40% of the minimum offset of 32
#define ARRAY_LENGTH                  (339)   // Twice sample length + max filter length plus 1 MUST BE ODD for sum of squares calculation and 2*SAMPLE_LENGTH+MAX_FILTER_LENGTH must be divisible by 2^DOWNSAMPLE_BITS
#define MIN_HR_PERIOD                 (32)
#define MAX_HR_PERIOD                 (148)
#define MIN_TRIANGLE_HALF_LENGTH      (4)
#define SAMPLE_RATE                   (100)   // 100 samples per second
#define MIN_SPO2                      (70)
#define SAMPLE_RATE_PER_MIN           (6000)  // Samples per minute
#define DEFAULT_GUESS                 (80)    // 75 BPM at 100sps
#define DOWNSAMPLE_BITS               (2)     // Number of bits defining the reduction in data rate for slope estimation (e.g. 4x downsampling is 2 bits)
#define SMALL_STEP                    (1)
#define MID_STEP                      (2)
#define BIG_STEP                      (2)
#define FIXED_BITS                    (3)     // For basic fixed precision enhancement. Don't change this without looking carefully at spo2.corr function >>2 instances and potential overflows.
#define IR_1                          (1)     // Channel definitions. Can't switch these (might seem illogical because IR data comes in first) because when we cycle through channels the last one to be analyzed with DC removal is IR, which is used for HR calculations
#define RED                           (0)
#define FILTER_BITS                   (5)     // 32 (for fastAvg2Nsamples filter)-not currently in use
#define MAX_FINE_STEP                 (3)     // Maximum step to be used in fine search for slow heart beats
#define DEFAULT_EST_PERIOD_1F         (1280)  // 75 BPM

//ALG Part 3 defines
#define PI_CALIBRATION                (14)
#define PEAK_SENSITIVITY              (8)     // The fraction of the max-min hr value in the buffer that will detect a peak e.g. 10 means (max-min)/10 is large enough to be considered a peak
#define USE_DISPLAY_AVG
#define DISPLAY_NUM2AVG               (38)    // (Max 60) Avg's samples on top of the previous Kalman average. Duration is DISPLAY_NUM2AVG * INTERVAL/SAMPLE_RATE
#define NUM_RR_FILTERS                (5)
#define MAX_BREATH_FILTER_LENGTH      (28)    // This sets the minimum detectable breathing rate 60/(interval/sample_rate*MBFL)
#define MAX_BREATH_FILTER_SPAN        (45)    // 1.6xMAX_BREATH_FILTER_LENGTH
#define MAX_BREATH_ARRAY_LENGTH       (73)    // 1.6 times the max breath filter (51) length + (max breath filter length (32)-1) (32 data points get us the first sample, we only need to add 50 more).

// This collects breathing rate data in about 32 seconds after the first B2B sample, or 36-37 seconds total.
#define MAX_TRIANGLE_FILTER_LENGTH    (MAX_OFFSET)  // The longest FIR filter we would use for B2B is the MAX_OFFSET
#define DO_RESET                      (0)
#define DO_AVG                        (1)
#define BINS                          (8)           // For use in cheap display average (average of averages)
#define BIN_SIZE                      (8)           // Ditto
#define FILTER_DAMPING                (3)           // Increasing this will increase the time to find the correct signal but it should also increase the stability
#define IIR_DEFAULT_DIV               (12)
#define IIR_MAX_DELTA                 (2500)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern uint16_t data_ptr;
extern uint16_t prev_data_ptr;
extern int32_t mean1f[2];
extern float R;                                 // R value used for SpO2 calculation
extern uint32_t rms1f[2];                       // Rms value in fixed precision
extern float perfusion_index[2];

extern uint16_t current_spo21f;                 // Most recent SpO2 calculation in percent and fixed precision
extern uint16_t current_hr1f;                   // Most recent heart rate value in BPM and fixed precision
extern uint16_t avg_hr1f;                       // Average heart rate determined by Kalman filter to output to user (fixed precision)
extern uint16_t avg_spo21f;                     // Average Spo2 value determined by Kalman filter to output ot user (fixed precision)
extern volatile uint16_t sample_count;          // Keeps track of whether the sample buffer is full or not. If full then alg runs.
extern volatile uint32_t read_sample_count;     // Keeps track of total number of samples collected
extern volatile uint32_t missed_sample_count;
extern uint32_t prev_sample_count;
extern uint32_t prev_missed_samples;
extern uint16_t read_samples;
extern uint16_t missed_samples;
extern int32_t alg_start_sample_count;          // Records the number most recent sample when the algorithm starts
extern int32_t prev_alg_start_sample_count;     // Remembers the sample count start position of the previous alg run
extern int16_t b2b1f;                           // Beat 2 beat time in # of samples
extern int16_t ppg_falltime;                    // Ppg fall time in # of samples
extern volatile uint8_t reset_kalman_hr;        // Flag to reset kalman filter for HR
extern volatile uint8_t reset_kalman_spo2;      // Flag to reset kalman filter for SpO2
extern uint16_t p2_start_time;                  // Tracking when the algorithm starts to check for timeout
extern uint16_t display_spo2;                   // Value output to display
extern uint16_t display_hr;                     // Value output to display
extern int32_t fit_correl;                      // Estimated correlation value from quadratic fit function.
extern uint8_t samples2avg;                     // Length of moving average filter for heart rate calculations
extern uint16_t rr_time1f;                      // Fixed precision output from peak find beat to beat time --FIGURE OUT A WAY TO PUBLISH THIS DATA. IT IS ASYNCHROUS.
extern int32_t peak_amp;                        // From simple_peak_find and get_peak_height
extern int32_t hr_data_buffer[MAX_BREATH_ARRAY_LENGTH];
extern uint8_t hr_data_buffer_ind;
extern uint32_t breathing_rate1f;               // Breaths per minute with fixed precision
extern int16_t iir_avg[NUM_RR_FILTERS];
extern int16_t iir_rms[NUM_RR_FILTERS];
extern int16_t iir_rms_smooth[NUM_RR_FILTERS];
extern uint8_t rr_filter_lens[NUM_RR_FILTERS];  // Values are set in the class init function
extern int alg_count;
extern int period1f;
extern uint8_t res_avg;                         // For resetting the 30sec RR average (after a finger was removed, e.g.)
extern int16_t final_offset1f;
extern uint32_t first_data;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief         Spo2 init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_init(void);

/**
 * @brief 
 * 
 * This function calculates the sum of squares used in the first order regression
 * in get_rms(). It is run once at the beginning of the program unless the sample
 * length is changed. Currently it is fixed. The method assumes the indices are centered
 * on zero and calculates only one half and doubles it. For this reason we use an odd
 * number of samples. Otherwise this would be fraction arithmetic as the indices
 * would be -1/2, 1/2....
 * This function also calculates the square values and stores them in idx2 and
 * uses those squares to calculated the sum of fourth power terms used in the 
 * calculation of the second order regression. See the Matlab code for notes.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_get_sum_squares(void);

/**
 * @brief 
 * 
 * This gets the mean levels, copies AC values into arrays and does DC and slope removal
 * variance or RMS calculation for SpO2 if there are enough samples.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */

void spo2_do_algorithm_part1(void);

/**
 * @brief 
 * 
 * This runs the SpO2 calculation and heart rate calculation. The algorithm
 * was split in half to allow a non-RTOS machine to take a break to collect samples
 * from the OB1203 buffer. SpO2 is calculated with the R values REDrms/REDmean / IRrms/IRmean.
 * Heart rate is calculated by finding the first peak of the autocorrelation using
 * variable stepping to find the minimum and then doing fine search at twice the minimum.
 * Heart rate-dependent filtering is applied to the AC IR data.
 * The stepping in the search algorithm is small for small offsets and large for
 * large offsets to keep accuracy proportional and minimize the number of steps.
 * The correlations is sample_length multply+add operations, so it takes time.
 * Time limits have been applied to time_out in time to get samples from the buffer.
 * A key feature is the Kalman filter which throws out outliers by comparing the recent sample
 * standard deviation to the next sample. Samples twice the stdev are tossed and not
 * included in the kalman running average estimation, but are included in the variance 
 * calculation. If too many algorithm fails or outliers are incurred the Kalman resets.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_do_algorithm_part2(void);

/**
 * @brief         This function does the breathing rate detection by detecting periodic variations in heart rate signals.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_do_algorithm_part3(void);

/**
 * @brief          USE A VARIETY OF TRIANGLE FILTERS TO FILTER THE DATA AND GET SEVERAL ESTIMATES OF THE RR PERIOD
 * 
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_do_algorithm_part4(void);

/**
 * @brief          Called by main to load new samples into the algorithm's buffer.
 *                 Removes the large DC offset TARGET_COUNTS to store 18 bit unsigned as int16
 * 
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void spo2_add_sample(uint32_t ir_data, uint32_t r_data);

/**
 * @brief         Uint sqrt
 * 
 * @param[in]     val     Value
 *
 * @attention     None
 *
 * @return        None
 */
uint32_t uint_sqrt(uint32_t val);

/**
 * @brief         Uint64 sqrt
 * 
 * @param[in]     val     Value
 *
 * @attention     None
 *
 * @return        None
 */
uint32_t uint64_sqrt(uint64_t val);

#endif // __SPO2_H

/* End of file -------------------------------------------------------- */
