/**
 * @file       ob1203_defs.h
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
#ifndef __OB1203_DEFS_H
#define __OB1203_DEFS_H

/* Includes ----------------------------------------------------------- */
/* Public defines ----------------------------------------------------- */
#define OB1203_I2C_ADDR                 (0xA6 >> 1)

//Define registers
#define OB1203_REG_STATUS_0             (0x00)
#define OB1203_REG_STATUS_1             (0x01)
#define OB1203_REG_PS_DATA              (0x02)
#define OB1203_REG_LS_W_DATA            (0x04)
#define OB1203_REG_LS_G_DATA            (0x07)
#define OB1203_REG_LS_B_DATA            (0x0A)
#define OB1203_REG_LS_R_DATA            (0x0D)
#define OB1203_REG_LS_C_DATA            (0x10)
#define OB1203_REG_TEMP_DATA            (0x13)
#define OB1203_REG_MAIN_CTRL_0          (0x15)
#define OB1203_REG_MAIN_CTRL_1          (0x16)
#define OB1203_REG_PS_LED_CURR          (0x17)
#define OB1203_REG_PS_CAN_PULSES        (0x19)
#define OB1203_REG_PS_PWIDTH_RATE       (0x1A)
#define OB1203_REG_PS_CAN_DIG           (0x1B)
#define OB1203_REG_PS_MOV_AVG_HYS       (0x1D)
#define OB1203_REG_PS_THRES_HI          (0x1E)
#define OB1203_REG_PS_THRES_LO          (0x20)
#define OB1203_REG_LS_RES_RATE          (0x22)
#define OB1203_REG_LS_GAIN              (0x23)
#define OB1203_REG_LS_THRES_HI          (0x24)
#define OB1203_REG_LS_THRES_LO          (0x27)
#define OB1203_REG_LS_THRES_VAR         (0x2A)
#define OB1203_REG_INT_CFG_0            (0x2B)
#define OB1203_REG_PS_INT_CFG_1         (0x2C)
#define OB1203_REG_INT_PST              (0x2D)
#define OB1203_REG_PPG_PS_GAIN          (0x2E)
#define OB1203_REG_PPG_PS_CFG           (0x2F)
#define OB1203_REG_PPG_IRLED_CURR       (0x30)
#define OB1203_REG_PPG_RLED_CURR        (0x32)
#define OB1203_REG_PPG_CAN_ANA          (0x34)
#define OB1203_REG_PPG_AVG              (0x35)
#define OB1203_REG_PPG_PWIDTH_RATE      (0x36)
#define OB1203_REG_FIFO_CFG             (0x37)
#define OB1203_REG_FIFO_WR_PTR          (0x38)
#define OB1203_REG_FIFO_RD_PTR          (0x39)
#define OB1203_REG_FIFO_OVF_CNT         (0x3A)
#define OB1203_REG_FIFO_DATA            (0x3B)
#define OB1203_REG_PART_ID              (0x3D)
#define OB1203_REG_DEVICE_CFG           (0x4D)
#define OB1203_REG_OSC_TRIM             (0x3E)
#define OB1203_REG_LED_TRIM             (0x3F)
#define OB1203_REG_BIO_TRIM             (0x40)
#define OB1203_REG_DIG_LED1_TRIM        (0x42)
#define OB1203_REG_DIG_LED2_TRIM        (0x43)

// Define settings
// STATUS_0
#define POR_STATUS                      (0x80)
#define LS_INT_STATUS                   (0x02)
#define LS_NEW_DATA                     (0x01)

// STATUS_1
#define LED_DRIVER_STATUS               (0x40)
#define FIFO_AFULL_STATUS               (0x20)
#define PPG_DATA_STATUS                 (0x10)
#define PS_LOGIC_STATUS                 (0x04)
#define PS_INT_STATUS                   (0x02)
#define PS_NEW_DATA                     (0x01)

// MAIN_CTRL_0
#define SW_RESET                        (0x01 << 7)
#define LS_SAI_ON                       (0x01 << 3)
#define LS_SAI_OFF                      (0x00)
#define ALS_MODE                        (0x00)
#define RGB_MODE                        (0x01 << 1)
#define LS_OFF                          (0x00)
#define LS_ON                           (0x01)

// MAIN_CTRL_1
#define PS_SAI_ON                       (0x01 << 3)
#define PS_SAI_OFF                      (0x00)
#define PS_MODE                         (0x00)
#define HR_MODE                         (0x01 << 1)
#define SPO2_MODE                       (0x02 << 1)
#define PPG_PS_ON                       (0x01)
#define PPG_PS_OFF                      (0x00)
#define TEMP_ON                         (0x01 << 7)
#define TEMP_OFF                        (0x00)

// PS_CAN_PULSES
#define PS_CAN_ANA_0                    (0x00)                // Off
#define PS_CAN_ANA_1                    (0x01 << 6)           // 50% of FS
#define PS_CAN_ANA_2                    (0x02 << 6)           // 100% of FS
#define PS_CAN_ANA_3                    (0x03 << 6)           // 150% of FS
#define PS_PULSES(x)                    (((x) & (0x07)) << 3) // Where x = 0..5 and num pulses = 2^x

// PS_PWIDTH_RATE
#define PS_PWIDTH(x)                    ((x & 0x03) << 4)     // Where x = 0..3
#define PS_RATE(x)                      ((x & 0x07))          // Where x = 0..7

// PS_MOV_AVG_HYS
#define PS_AVG_ON                       (1 << 7)
#define PS_AVG_OFF                      (0)
#define PS_HYS_LEVEL(x)                 (x >> 1)              // Where x = 0..256

// LS_RES_RATE
#define LS_RES(x)                       ((x & 0x07) << 4)     // Where x = 0..7
#define LS_RATE(x)                      ((x & 0x07))          // Where x = 0..7
#define LS_RES_20                       (0x00)
#define LS_RES_19                       (0x01)
#define LS_RES_18                       (0x02)
#define LS_RES_17                       (0x03)
#define LS_RES_16                       (0x04)
#define LS_RES_13                       (0x05)

// LS_GAIN
#define LS_GAIN(x)                      (x & 0x03)
#define LS_GAIN_1                       (0x00)
#define LS_GAIN_3                       (0x01)
#define LS_GAIN_6                       (0x10)
#define LS_GAIN_20                      (0x11)

// LS_THRES_VAR
#define LS_THRES_VAR(x)                 (x & 0x07)

// INT_CFG_0
#define LS_INT_SEL_W                    (0)
#define LS_INT_SEL_G                    (1 << 4)
#define LS_INT_SEL_R                    (2 << 4)
#define LS_INT_SEL_B                    (3 << 4)
#define LS_THRES_INT_MODE               (0)
#define LS_VAR_INT_MODE                 (1 << 1)
#define LS_INT_ON                       (1)
#define LS_INT_OFF                      (0)

// INT_CFG_1
#define AFULL_INT_ON                    (1 << 5)
#define AFULL_INT_OFF                   (0)
#define PPG_INT_ON                      (1 << 4)
#define PPG_INT_OFF                     (0)
#define PS_INT_READ_CLEARS              (0 << 1)
#define PS_INT_LOGIC                    (1)
#define PS_INT_ON                       (1)
#define PS_INT_OFF                      (0)

// INT_PST
#define LS_PERSIST(x)                   ((x & 0x0F) << 4)
#define PS_PERSIST(x)                   (x & 0x0F)

// PPG_PS_GAIN
#define PPG_PS_GAIN_1                   (0)
#define PPG_PS_GAIN_1P5                 (1 << 4)
#define PPG_PS_GAIN_2                   (2 << 4)
#define PPG_PS_GAIN_4                   (3 << 4)
#define PPG_LED_SETTLING(x)             ((x & 0x03) << 2) // 0 = 0us, 1 = 5us, 2 = 10us (Default), 3=20us
#define PPG_ALC_TRACK(x)                (x & 0x03)        // 0 = 10us, 1 (20us) DEFAULT ,2 = 30us, 3 = 60us

// PPG_PS_CFG
#define PPG_POW_SAVE_ON                 (1 << 6)
#define PPG_POW_SAVE_OFF                (0)
#define LED_FLIP_ON                     (1 << 3)
#define LED_FLIP_OFF                    (0)
#define DIFF_OFF                        (2)
#define ALC_OFF                         (1)
#define DIFF_ON                         (0)
#define ALC_ON                          (0)
#define SIGNAL_OUT                      (1 << 2)
#define OFFSET_OUT                      (0)

// PPG_CAN_ANA
#define PPG_CH1_CAN(x)                  ((x & 0x03) << 2)
#define PPG_CH2_CAN(x)                  (x & 0x03)

// PPG_AVG
#define PPG_AVG(x)                      ((x & 0x07) << 4)

// PPG_PWIDTH_RATE
#define PPG_PWIDTH(x)                   (x & 0x07) << 4
#define PPG_FREQ_PRODUCTION             (0)
#define PPG_FREQ_PREPRODUCTION          (1 << 3)
#define PPG_RATE(x)                     (x & 0x07)

// FIFO_CFG
#define FIFO_ROLL_ON                    (1<<4)
#define FIFO_ROLL_OFF                   (0)
#define AFULL_ADVANCE_WARNING(x)        (x & 0x0F)

#define POR_TIME_MS                     (10)
#define TARGET_COUNTS                   (0x30000)      // 196kcounts (3/4 * 2^18)
#define TOL1                            (6000)
#define TOL2                            (0x8000)       // 32k counts (+/-2^15) -- so we can represent values as 18 bit data minus setpoint in int16_t
#define STEP                            (8)
#define IN_RANGE_PERSIST                (4)
#define IR_MAX_CURRENT                  (0x02AF)
#define R_MAX_CURRENT                   (0x01FF)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif // __OB1203_DEFS_H

/* End of file -------------------------------------------------------- */
