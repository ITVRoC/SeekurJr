/****************************************************************************
 * Copyright (c) 2007 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Wed Mar 28 2007
 * 
 * Brief Description:
 * This file contains addresses and bitfield descriptions for regsiters in the
 * Micron MT9V022 image sensor.
 * 
 * Functionality:
 * 
 * Issues:
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/
#ifndef FREAGLESTEREOCAMREGS_H
#define FREAGLESTEREOCAMREGS_H

// Offsets for the MT9V022 Registers
#define MT9V022_CHIP_VERSION                    0x00
#define MT9V022_COLUMN_START                    0x01
#define MT9V022_ROW_START                       0x02
#define MT9V022_WINDOW_HEIGHT                   0x03
#define MT9V022_WINDOW_WIDTH                    0x04
#define MT9V022_HORIZONTAL_BLANKING             0x05
#define MT9V022_VERTICAL_BLANKING               0x06
#define MT9V022_CHIP_CONTROL                    0x07
#define MT9V022_SHUTTER_WIDTH_1                 0x08
#define MT9V022_SHUTTER_WIDTH_2                 0x09
#define MT9V022_SHUTTER_WIDTH_CTRL              0x0a
#define MT9V022_TOTAL_SHUTTER_WIDTH             0x0b
#define MT9V022_RESET                           0x0c
#define MT9V022_READ_MODE                       0x0d
#define MT9V022_MONITOR_MODE                    0x0e
#define MT9V022_PIXEL_OPERATION_MODE            0x0f
#define MT9V022_LED_OUT_CTRL                    0x1b
#define MT9V022_ADC_MODE_CONTROL                0x1c
#define MT9V022_VREF_ADC_CONTROL                0x2c
#define MT9V022_V1                              0x31
#define MT9V022_V2                              0x32
#define MT9V022_V3                              0x33
#define MT9V022_V4                              0x34
#define MT9V022_ANALOG_GAIN                     0x35
#define MT9V022_MAX_ANALOG_GAIN                 0x36
#define MT9V022_FRAME_DARK_AVERAGE              0x42
#define MT9V022_DARK_AVG_THRESHOLDS             0x46
#define MT9V022_BL_CALIB_CONTROL                0x47
#define MT9V022_BL_CALIBRATION_VALUE            0x48
#define MT9V022_BL_CALIB_STEP_SIZE              0x4c
#define MT9V022_FUSE_WORD_1                     0x68
#define MT9V022_FUSE_WORD_2                     0x69
#define MT9V022_FUSE_WORD_3                     0x6A
#define MT9V022_FUSE_WORD_4                     0x6B
#define MT9V022_ROW_NOISE_CORR_CTRL_1           0x70
#define MT9V022_ROW_NOISE_CONSTANT              0x72
#define MT9V022_ROW_NOISE_CORR_CTRL_2           0x73
#define MT9V022_DIGITAL_TEST_PATTERN            0x7F
#define MT9V022_TILED_DIGITAL_GAIN_X0Y0         0x80
#define MT9V022_TILED_DIGITAL_GAIN_X2Y2         0x8C
#define MT9V022_TILED_DIGITAL_GAIN_X4Y4         0x98

#define MT9V022_DIGITAL_TILE_COORD_1X           0x99
#define MT9V022_DIGITAL_TILE_COORD_2X           0x9A
#define MT9V022_DIGITAL_TILE_COORD_3X           0x9B
#define MT9V022_DIGITAL_TILE_COORD_4X           0x9C
#define MT9V022_DIGITAL_TILE_COORD_5X           0x9D
#define MT9V022_DIGITAL_TILE_COORD_6X           0x9E
#define MT9V022_DIGITAL_TILE_COORD_1Y           0x9F
#define MT9V022_DIGITAL_TILE_COORD_2Y           0xA0
#define MT9V022_DIGITAL_TILE_COORD_3Y           0xA1
#define MT9V022_DIGITAL_TILE_COORD_4Y           0xA2
#define MT9V022_DIGITAL_TILE_COORD_5Y           0xA3
#define MT9V022_DIGITAL_TILE_COORD_6Y           0xA4
#define MT9V022_AEC_AGC_DESIRED_BIN             0xA5
#define MT9V022_AEC_UPDATE_FREQUENCY            0xA6
#define MT9V022_AEC_LPF                         0xA8
#define MT9V022_AGC_UPDATE_FREQUENCY            0xA9
#define MT9V022_AGC_LPF                         0xAB
#define MT9V022_AEC_AGC_ENABLE                  0xAF
#define MT9V022_AEC_AGC_PIX_COUNT               0xB0
#define MT9V022_LVDS_MASTER_CTRL                0xB1
#define MT9V022_LVDS_SHIFT_CLK_CTRL             0xB2
#define MT9V022_LVDS_DATA_CTRL                  0xB3
#define MT9V022_DATA_STREAM_LATENCY             0xB4
#define MT9V022_LVDS_INTERNAL_SYNC              0xB5
#define MT9V022_LVDS_PAYLOAD_CONTROL            0xB6
#define MT9V022_STEREOSCOP_ERROR_CTRL           0xB7
#define MT9V022_STEREOSCOP_ERROR_FLAG           0xB8
#define MT9V022_LVDS_DATA_OUTPUT                0xB9
#define MT9V022_AGC_GAIN_OUTPUT                 0xBA
#define MT9V022_AEC_EXPOSURE_OUTPUT             0xBB
#define MT9V022_AGC_AEC_CURRENT_BIN             0xBC
#define MT9V022_MAXIMUM_SHUTTER_WIDTH           0xBD
#define MT9V022_AGC_AEC_BIN_DIFFERENCE_THRESHOLD 0xBE
#define MT9V022_FIELD_BLANK                     0xBF
#define MT9V022_MON_MODE_CAPTURE_CTRL           0xC0
#define MT9V022_TEMPERATURE                     0xC1
#define MT9V022_ANALOG_CONTROLS                 0xC2
#define MT9V022_BYTEWISE_ADDR                   0xF0
#define MT9V022_REGISTER_LOCK                   0xFE
#define MT9V022_ALT_CHIP_VERSION                0xFF

#define I2C_DEVICE_WINDOW_HEIGHT_ADDR		3
#define I2C_DEVICE_CHIP_CONTROL                 7
#define I2C_DEVICE_SHUTTER_WIDTH_CTRL		10
#define I2C_DEVICE_RESET_ADDR                   12
#define I2C_DEVICE_PIXEL_MODE_ADDR              15
#define I2C_DEVICE_ADC_RESOLUTION_ADDR		28
#define I2C_DEVICE_ROW_NOISE_COR_1_ADDR         112
#define I2C_DEVICE_ROW_NOISE_COR_2_ADDR         115
#define I2C_DEVICE_TEST_PATTERN_ADDR            127
#define I2C_DEVICE_AGC_AEC_DESIRED_BIN_ADDR     165
#define I2C_DEVICE_AGC_AEC_ENABLE               175
#define I2C_DEVICE_MASTER_CONTROL_ADDR		177
#define I2C_DEVICE_SCLK_CONTROL_ADDR		178
#define I2C_DEVICE_DATA_CONTROL_ADDR		179
#define I2C_DEVICE_INTERNAL_SYNC_ADDR           181
#define I2C_DEVICE_STEREO_ERR_CTRL_ADDR		183
#define I2C_DEVICE_STEREO_ERR_FLAG_ADDR		184
#define I2C_DEVICE_LVDS_DATA_OUTPUT_ADDR	185
#define I2C_DEVICE_ANALOG_CONTROL_ADDR          194


#define I2C_DEVICE_RESERVED_10_ADDR             16
#define I2C_DEVICE_RESERVED_15_ADDR		21
#define I2C_DEVICE_RESERVED_20_ADDR             32
#define I2C_DEVICE_RESERVED_21_ADDR             33



// MT9V022 CSRS Bitfields
typedef union {
    struct lvds_sensor_reset {
	unsigned int soft_reset			: 1; /* 0 */
	unsigned int auto_block_soft_reset      : 1; /* 1 */
	unsigned int				: 14;
    } bit;
    unsigned int val;
} lvds_sensor_reset_u;

typedef union {
    struct lvds_sensor_master_control {
	unsigned int pll_bypass			: 1; /* 0 */
	unsigned int lvds_power_down		: 1; /* 1 */
	unsigned int pll_test_mode              : 1; /* 2 */
	unsigned int lvds_test_mode             : 1; /* 3 */
	unsigned int				: 12;
    } bit;
    unsigned int val;
} lvds_sensor_master_control_u;

typedef union {
    struct lvds_sensor_sclk_control {
	unsigned int delay		        : 3; /* 2:0 */
	unsigned int				: 1; /*  3  */
	unsigned int power_down			: 1; /*  4  */
	unsigned int				: 11;
    } bit;
    unsigned int val;
} lvds_sensor_sclk_control_u;

typedef union {
    struct lvds_sensor_data_control {
	unsigned int delay		      : 3; /* 2:0 */
	unsigned int                          : 1; /*  3  */
	unsigned int power_down		      : 1; /*  4  */
	unsigned int			      : 11;
    } bit;
    unsigned int val;
} lvds_sensor_data_control_u;

typedef union {
    struct lvds_sensor_internal_sync {
	unsigned int sync_enable		: 1; /* 0 */
	unsigned int				: 15;
    } bit;
    unsigned int val;
} lvds_sensor_internal_sync_u;

typedef union {
    struct lvds_sensor_agc_aec_enable {
	unsigned int aec_enable			: 1; /* 0 */
	unsigned int agc_enable			: 1; /* 1 */
	unsigned int				: 14;
    } bit;
    unsigned int val;
} lvds_sensor_agc_aec_enable_u;


typedef union {
    struct lvds_sensor_row_noise_cor_1 {
	unsigned int number_dark_pixels		: 4; /* 3:0 */
	unsigned int				: 1; /* 4 */
	unsigned int enable_noise_correction    : 1; /* 5 */
	unsigned int                            : 5; /* 10:6 */
	unsigned int use_black_level_average    : 1; /* 11 */
	unsigned int                            : 4;
    } bit;
    unsigned int val;
} lvds_sensor_row_noise_cor_1_u;

typedef union {
    struct lvds_sensor_row_noise_cor_2 {
	unsigned int dark_column_start_addr     : 10; /* 9:0 */
	unsigned int                            : 6; 
    } bit;
    unsigned int val;
} lvds_sensor_row_noise_cor_2_u;

typedef union {
    struct lvds_sensor_analog_control {
	unsigned int				: 7; /* 6:0 */ 
	unsigned int anti_eclipse_enable	: 1; /* 7 */
	unsigned int                            : 3; /* 10:8 */
	unsigned int v_rst_lim_voltage_level    : 3; /* 13:11 */
	unsigned int                            : 2;
    } bit;
    unsigned int val;
} lvds_sensor_analog_control_u;


typedef union {
    struct lvds_sensor_test_pattern {
	unsigned int two_wire_test_data         : 10;  /* 9:0 */
	unsigned int two_wire_test_enable       : 1;   /* 10 */
	unsigned int gray_shade_test_pattern    : 2;   /* 12:11 */
        unsigned int test_enable                : 1;   /* 13 */
	unsigned int flip_two_wire_test_data    : 1;   /* 14 */
	unsigned int				: 1;
    } bit;
    unsigned int val;
} lvds_sensor_test_pattern_u;

typedef union {
    struct lvds_sensor_chip_control {
	unsigned int scan_mode                  : 3;  /* 2:0 */
	unsigned int sensor_master_slave_mode   : 1;  /* 3 */
	unsigned int sensor_snapshot_mode	: 1;  /* 4 */
	unsigned int stereo_mode		: 1;  /* 5 */
	unsigned int stereo_master_slave_mode   : 1;  /* 6 */
	unsigned int parallel_output_enable     : 1;  /* 7 */
	unsigned int simul_sequ_mode            : 1;  /* 8 */
	unsigned int defect_pixel_correctoin	: 1;  /* 9 */
	unsigned int                            : 6;
    } bit;
    unsigned int val;
} lvds_sensor_chip_control_u;

typedef union {
    struct lvds_sensor_reserved_core_10 {	
	unsigned int fpn_fix			: 7; /* 6:0 */
	unsigned int				: 9;
    } bit;
    unsigned int val;
} lvds_sensor_reserved_core_10_u;

typedef union {
    struct lvds_sensor_reserved_core_15 {	
	unsigned int                            : 8; /* 7:0 */
	unsigned int fpn_fix			: 8; /* 15:8 */
    } bit;
    unsigned int val;
} lvds_sensor_reserved_core_15_u;

typedef union {
    struct lvds_sensor_reserved_core_20 {	
	unsigned int                            : 6; /* 5:0 */
	unsigned int fpn_fix			: 3; /* 8:6 */
	unsigned int                            : 7;
    } bit;
    unsigned int val;
} lvds_sensor_reserved_core_20_u;

typedef union {
    struct lvds_sensor_reserved_core_21 {	
	unsigned int fpn_fix			: 6; /* 5:0 */
	unsigned int				: 10;
    } bit;
    unsigned int val;
} lvds_sensor_reserved_core_21_u;

typedef union {
    struct lvds_sensor_pixel_mode {	
	unsigned int				: 2; /* 1:0 */
	unsigned int color_mono			: 1; /* 2 */
	unsigned int				: 3; /* 5:3 */
	unsigned int hidy_enable		: 1; /* 6 */
	unsigned int extended_exposure_enable   : 1; /* 7 */
	unsigned int				: 8; 							
    } bit;
    unsigned int val;
} lvds_sensor_pixel_mode_u;

typedef union {
    struct lvds_sensor_agc_aec_desired_bin {	
	unsigned int desired_bin		: 6; /* 5:0 */
	unsigned int				: 10;
    } bit;
    unsigned int val;
} lvds_sensor_agc_aec_desired_bin_u;

typedef union {
    struct lvds_sensor_shutter_width {	
	unsigned int shutter_width		: 15; /* 14:0 */
	unsigned int				: 1;
    } bit;
    unsigned int val;
} lvds_sensor_shutter_width_u;

typedef union {
    struct lvds_sensor_shutter_width_ctrl {	
      unsigned int t2_ratio			: 4; /* 3:0 */
      unsigned int t3_ratio			: 4; /* 7:4 */
      unsigned int knee_auto_en			: 1; /* 8 */
      unsigned int single_knee_en		: 1; /* 9 */
      unsigned int				: 6; 
    } bit;
    unsigned int val;
} lvds_sensor_shutter_width_ctrl_u;


typedef union {
    struct lvds_sensor_adc_resolution {	
	unsigned int adc_mode			: 2; /* 1:0 */
	unsigned int				: 14;
    } bit;
    unsigned int val;
} lvds_sensor_adc_resolution_u;


typedef union {
    struct lvds_sensor_lvds_data_output {	
	unsigned int combo_reg			: 16; /* 15:0 */
    } bit;
    unsigned int val;
} lvds_sensor_lvds_data_output_u;

typedef union {
    struct lvds_sensor_stereo_err_ctrl {	
      unsigned int enable_error_detect		: 1; 
      unsigned int enable_stick_flag		: 1;
      unsigned int clear_error_flag		: 1;
      unsigned int				: 13; /* 13:0 */
    } bit;
    unsigned int val;
} lvds_sensor_stereo_err_ctrl_u;

typedef union {
  struct lvds_sensor_stereo_err_flag {
    unsigned int error_flag			: 1;
    unsigned int				: 15;
  } bit;
  unsigned int val;
} lvds_sensor_stereo_err_flag_u;


#endif
