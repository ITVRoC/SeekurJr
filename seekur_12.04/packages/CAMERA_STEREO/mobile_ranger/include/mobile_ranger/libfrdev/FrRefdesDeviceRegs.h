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

#ifndef FRREFDESDEVICEREGS_H
#define FRREFDESDEVICEREGS_H

// Size of csr and memory address space for mmap
#define REFDES_CSR_SPACE_LENGTH     0x00100000 // size 1MB
#define REFDES_MEMORY_SPACE_LENGTH  0x02000000 // size 32MB

// Reference Design  CSR offsets
// Naming convention is : REFDES_CSR_[GROUP]_[FIELD]
#define REFDES_CSR_CHIP_VERSION			0
  
// csr to set basic image properties
#define REFDES_CSR_IMAGE_HEIGHT			1
#define REFDES_CSR_IMAGE_WIDTH			2
#define REFDES_CSR_IMAGE_FORMAT			3

// csr for vrx frames in sdram memory
#define REFDES_CSR_VRX_BASE_ADDR_CH0		4
#define REFDES_CSR_VRX_BASE_ADDR_CH1		5
#define REFDES_CSR_VRX_FRAME_CNT 		6 
#define REFDES_CSR_VRX_FRAME_ERROR              7
#define REFDES_CSR_VRX_LINE_CNT			8 
#define REFDES_CSR_VRX_FRAME_SKIP_N             9
#define REFDES_CSR_VRX_FRAME_SKIP_M             10

// csr for stereo lvds RX deserializer
#define REFDES_CSR_VRX_EN			11
#define REFDES_CSR_VRX_RPWDN			12
#define REFDES_CSR_VRX_LOCK			13
#define REFDES_CSR_VRX_ERROR			14
#define REFDES_CSR_VRX_ERROR_STATUS		15

// csr to read/write lvds sensor csrs
#define REFDES_CSR_I2C_DEVICE_CSR_START		16
#define REFDES_CSR_I2C_DEVICE_CSR_SLAVE_ID	17
#define REFDES_CSR_I2C_DEVICE_CSR_ADDR		18  
#define REFDES_CSR_I2C_DEVICE_CSR_RW		19  
#define REFDES_CSR_I2C_DEVICE_CSR_DATA		20  
#define REFDES_CSR_I2C_DEVICE_CSR_ERROR		21  
#define REFDES_CSR_I2C_DEVICE_CSR_BCAST         22  
#define REFDES_CSR_I2C_DEVICE_CSR_DEVSEL        23  

// Reset and clocks block
#define REFDES_CSR_RESET                        51
#define REFDES_CSR_DCM_STATUS			47

// User JTAG control/GPIO
#define REFDES_CSR_GPIO                         50

#define REFDES_CSR_VRX_STATUS			52

#define REFDES_CSR_DEBUG_CTL                    56
#define REFDES_CSR_DEBUG_DAT0                   57

#endif
