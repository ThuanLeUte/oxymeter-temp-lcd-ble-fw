/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2019 Renesas Electronics Corporation. All rights reserved.
*
************************************************************************************************************************
* This code was created by Dan Allen, 2020, Renesas Electronics America. It is intended for demonstration purposes only, not
* for production. No performance or suitability for any purpose including medical devices is guaranteed or claimed. For
* questions and inquiries visit Renesas.com

**********************************************************************************************************************/
#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "spo2.h"

#define NO_JUMPS 0
#define JUMPS_OK 1


#define RR_KALMAN_LENGTH 13 //5 second average
#define RR_DATA_LENGTH 30
#define RR_MIN_STD_1F 4<<FIXED_BITS //this is the breath period in intervals. FOr example 70 internal with 100 sample rate means we are looking for std_dev of less than 4 intervals or about 3 seconds * threshold.
#define RR_KALMAN_THRESHOLD_2X 5
#define RR_MAX_OUTLIER_COUNT 18
#define RR_MAX_ALG_FAIL_COUNT 18

#define MIN_DATA_STD 4<<FIXED_BITS
#define ALG_FAIL_TOLERANCE 3 //number of algorith fails we tolerate before resetting the kalman filter
#define OUTLIER_DATA_TOLERANCE 3 //number of outlier samples we reject before resetting the kalman filter

#define CORR_KALMAN_LENGTH 2
#define CORR_DATA_LENGTH 20
#define CORR_MIN_STD_1F 8<<FIXED_BITS
#define CORR_KALMAN_THRESHOLD_2X 5
#define CORR_MAX_OUTLIER_COUNT 10
#define CORR_MAX_ALG_FAIL_COUNT 10

#define HR_KALMAN_LENGTH 25//number of points to make a running average over
//#define HR_KALMAN_LENGTH 9 //number of points to make a running average over
#define HR_DATA_LENGTH 35
//#define HR_DATA_LENGTH 15
#define HR_MIN_STD_1F 8<<FIXED_BITS
#define HR_KALMAN_THRESHOLD_2X  5//the multiplier for the standard deviation to use for the kalman filter
#define HR_MAX_OUTLIER_COUNT 8
#define HR_MAX_ALG_FAIL_COUNT 8

#define SPO2_KALMAN_LENGTH 25
//#define SPO2_KALMAN_LENGTH 8
#define SPO2_DATA_LENGTH 25
//#define SPO2_DATA_LENGTH 8
#define SPO2_MIN_STD_1F 4<<FIXED_BITS
#define SPO2_KALMAN_THRESHOLD_2X 4
#define SPO2_MAX_OUTLIER_COUNT 8
#define SPO2_MAX_ALG_FAIL_COUNT 8

#define MAX_OUTLIER_COUNT 3
#define MAX_ALG_FAIL_COUNT 3

#define P_D_MAX_ALG_FAIL_COUNT 3
#define P_D_MAX_OUTLIER_COUNT 3
#define P_D_DATA_LENGTH 10
#define P_D_KALMAN_LENGTH 6
#define P_D_MIN_PER_STD 10 //percentage change out of 127--for use with run_kalman_ratio() function
#define P_D_KALMAN_THRESHOLD_2X 5
#define NO_RATIO 0
#define USE_RATIO 1

#define KF kalman_filters[num]


//make an array of structs for kalman

/*variables*/
typedef struct kalman {
	uint8_t kalman_length;
	uint16_t *kalman_array;
	uint8_t kalman_ind;
	uint16_t kalman_avg;
	uint16_t *data_array;
	uint8_t data_array_length;
	uint8_t data_ind;
	uint8_t outlier_cnt;
	uint8_t alg_fail_cnt;
	uint8_t max_outlier_cnt;
	uint8_t max_alg_fail_cnt;
	uint8_t max_kalman_length;
	uint8_t max_data_length;
	uint8_t min_data_std;
	uint8_t kalman_threshold_2x;
	uint8_t do_reset_kalman;
	uint16_t data_std;
	uint16_t big_avg;
	//  uint32_t data_std_out;
	//  uint32_t data_mean_out;
	uint8_t jumps_ok; //use this for spo2 to allow upward jumps of up to 2x the usual threshold during fast SpO2 recovery
}kalman_t;


//#ifdef CALC_DER
//	#define NUM_KALMAN_FILTERS 8 //corr, HR, spo2, RR, pd_r, pd_ir, pd_r_der, pd_ir_der
//	enum filters {corr_filt,hr_filt,spo2_filt,rr_filt,pd_r_k,pd_ir_k,pd_r_der,pd_ir_der};
//
//#else
#define NUM_KALMAN_FILTERS 4 //corr, HR, spo2, RR
	enum filters {corr_filt,hr_filt,spo2_filt,rr_filt};
//#endif

extern kalman_t kalman_filters[NUM_KALMAN_FILTERS]; //declare an array of kalman filter structs


/*functions*/
void init_kalman(kalman_t* kalman_filters);
void reset_kalman(uint8_t filter_num);
void run_kalman(uint8_t filter_num, uint32_t new_data, uint8_t ratio);
uint16_t get_std_dev(uint16_t *array, uint8_t array_length);
uint16_t get_avg(uint16_t *array, uint8_t array_length);
void get_big_avg(uint8_t filter_num);

#endif
