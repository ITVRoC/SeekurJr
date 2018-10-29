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

#ifndef FREAGLEDEVICEREGS_H
#define FREAGLEDEVICEREGS_H

// Size of csr and memory address space for mmap
// Addr space base offsets are for use with Eagle kernel driver
#define EAGLE_CSR_SPACE_LENGTH     0x00100000 // size 1MB
#define EAGLE_CSR_SPACE_BASE       0x02000000 // offset 32MB
#define EAGLE_MEMORY_SPACE_LENGTH  0x02000000 // size 32MB
#define EAGLE_MEMORY_SPACE_BASE    0x0 // offset 0
#define EAGLE_RECT_MAP_BASE        4*1024*1024

// Eagle csr offsets
// Naming convention is : EAGLE_CSR_[GROUP]_[FIELD]
#define EAGLE_CSR_CHIP_VERSION			0
  
// csr to set basic image properties
#define EAGLE_CSR_IMAGE_HEIGHT			1
#define EAGLE_CSR_IMAGE_WIDTH			2
#define EAGLE_CSR_IMAGE_FORMAT			3

// csr for vrx frames in sdram memory
#define EAGLE_CSR_VRX_BASE_ADDR_CH0		4
#define EAGLE_CSR_VRX_BASE_ADDR_CH1		5
#define EAGLE_CSR_VRX_FRAME_CNT 		6 
#define EAGLE_CSR_VRX_FRAME_ERROR               7
#define EAGLE_CSR_VRX_LINE_CNT			8 
#define EAGLE_CSR_VRX_FRAME_SKIP_N              9
#define EAGLE_CSR_VRX_FRAME_SKIP_M              10

// csr for stereo lvds RX deserializer
#define EAGLE_CSR_VRX_EN			11
#define EAGLE_CSR_VRX_RPWDN			12
#define EAGLE_CSR_VRX_LOCK			13
#define EAGLE_CSR_VRX_ERROR			14
#define EAGLE_CSR_VRX_ERROR_STATUS		15

// csr to read/write lvds sensor csrs
#define EAGLE_CSR_I2C_DEVICE_CSR_START		16
#define EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID	17
#define EAGLE_CSR_I2C_DEVICE_CSR_ADDR		18  
#define EAGLE_CSR_I2C_DEVICE_CSR_RW		19  
#define EAGLE_CSR_I2C_DEVICE_CSR_DATA		20  
#define EAGLE_CSR_I2C_DEVICE_CSR_ERROR		21  
#define EAGLE_CSR_I2C_DEVICE_CSR_BCAST          22  
#define EAGLE_CSR_I2C_DEVICE_CSR_DEVSEL         23  

// csr for Calibration block
#define EAGLE_CSR_CALIB_ENABLE                  24 // bit0=en, bit1=single
#define EAGLE_CSR_CALIB_STATUS                  25 // bit0=done, bit1=drop, bit2=overflow
#define EAGLE_CSR_CALIB_LEFT_SRC                26 // 6 bit base address
#define EAGLE_CSR_CALIB_RIGHT_SRC               27 // 6 bit base address
#define EAGLE_CSR_CALIB_LEFT_DST                28 // 6 bit base address
#define EAGLE_CSR_CALIB_RIGHT_DST               29 // 6 bit base address

// csr for Stereo block
#define EAGLE_CSR_STEREO_ENABLE                 30 // bit0=en, bit1=single
#define EAGLE_CSR_STEREO_STATUS                 31 // bit0=done, bit1=drop, bit2=overflow
#define EAGLE_CSR_STEREO_LEFT_SRC               32 // 6 bit base address
#define EAGLE_CSR_STEREO_RIGHT_SRC              33 // 6 bit base address
#define EAGLE_CSR_STEREO_DISP_DST               34 // 6 bit base address
#define EAGLE_CSR_STEREO_SEARCH_MODE            35 // 01 for right-left, 10 for left-right, 11 for both
#define EAGLE_CSR_STEREO_SEARCH_RANGE           36 // 7 bit disp range, 63 typ, 123 max

// CSRs for DMA Controller/PCI Master
#define EAGLE_CSR_DMAC_SRC0                     37
#define EAGLE_CSR_DMAC_SRC1                     38
#define EAGLE_CSR_DMAC_SRC2                     39
#define EAGLE_CSR_DMAC_SRC3                     40
#define EAGLE_CSR_DMAC_STATUS                   41
#define EAGLE_CSR_PCIM_DST0                     42
#define EAGLE_CSR_PCIM_DST1                     43
#define EAGLE_CSR_PCIM_DST2                     44
#define EAGLE_CSR_PCIM_DST3                     45

// csr to control access to secondary memory subsystem
#define EAGLE_CSR_SDRAM1_OWNER_SELECT		46
#define EAGLE_CSR_DCM_STATUS			47

// csr for interrupt status
#define EAGLE_CSR_INT_STAT			48
#define EAGLE_CSR_INT_MASK			49

// User JTAG control/GPIO
#define EAGLE_CSR_GPIO                          50

// Reset and clocks block
#define EAGLE_CSR_RESET                         51

#define EAGLE_CSR_VRX_STATUS			52
#define EAGLE_CSR_CALIB_COEFF_BURSTS            53
#define EAGLE_CSR_AEGC_ENABLE			54
#define EAGLE_CSR_AEGC_SKIP_FRAMES		55

#define EAGLE_CSR_DEBUG_CTL                     56
#define EAGLE_CSR_DEBUG_DAT0                    57

#define EAGLE_CSR_VTX_CTL                       58
#define EAGLE_CSR_STEREO_MQC_CTL                59 // 3:0 shift, 7:4 man, 8 en

#endif

