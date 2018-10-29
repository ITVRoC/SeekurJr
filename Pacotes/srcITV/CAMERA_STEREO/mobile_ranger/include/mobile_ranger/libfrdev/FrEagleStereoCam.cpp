/****************************************************************************
 * Copyright (c) 2007-2009 by Focus Robotics.
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
 * Implementation of the FrEagleStereoCam class.
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
#include <stdio.h>  // for printf
#include <stdlib.h> // for exit

#include "FrEagleStereoCam.h"

/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * Constructor/Destructor
 ******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/**
 * Default constructor. Requires a pointer to an FrEagleDevice.
 */
FrEagleStereoCam::FrEagleStereoCam(FrEagleDevice *d) {
  dev = d;
  // FIXME: actually check for this! Assume it's always present for now.
  // There should probably be a routine to deal with camPresent==false instead of
  // just continually trying to restart them if they aren't there. Some sort of
  // i2c ping test.
  camPresent = true; // check for camera communication!!!
  selectedCam = -1; // broadcast by default, depreciated function
  aegcMode = 0; // manual by default
}

/**
 * Destructor. Clean up everything and shut down camera as much as possible.
 */
FrEagleStereoCam::~FrEagleStereoCam() {
  resetCameras();
}

/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * Basic Access
 ******************************************************************************/
/******************************************************************************/
/******************************************************************************/


/**
 * Sets the default sensor that future operations will work on. Choices are
 * 0 for master, 1 for slave, or -1 for both (only writes can be done to both)
 */
void FrEagleStereoCam::setCamSel(int sel) {
  selectedCam = sel;
}

/**
 * Sets a control or status register in an MT9V022 image sensor chip via the i2c
 * bus driven from an eagle device. Writes may be performed on either the master
 * or slave image sensor, or broadcast to both sensors at the same time. Which
 * sensor the operation will be performed on may be specified as part of the 
 * function call, or the default may be set with the setCamSel() call.
 */
void FrEagleStereoCam::setCamCSR(int offset, int data, int sel) {
  if(!camPresent) {
    printf("EagleStereoCam: setCamCSR ignored: no camera present.\n");
    return; // There's no way to really report the error--it should be an exception
  }
  int selCam = (sel==-2) ? selectedCam : sel;

  // FIXME: Attempted reimplementation of I2C interface is broken in edev
  /*
  int device = 0xB8;
  if(selCam==0) device = 0xB0;
  else if(selCam==-1) dev->setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 1);
  if(dev->writeWordI2C(offset, data, device)) {
    // The I2C transfer failed! either retry or clear the camera present flag.
    printf("EagleStereoCam: I2C write transaction to camera failed!\n");
    printf("EagleStereoCam: Disabling further transactions.");
    camPresent = false;
  }
  dev->setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 0);
  */
  dev->setI2CCSR(offset, data, selCam);
}


/**
 * Reads and returns the value from a control or status register in an MT9V022 
 * image sensor chip via the i2c bus driven from an eagle device. Reads may be
 * performed on either the master or slave sensor of the stereo pair. Which 
 * sensor the operation will be performed on may be specified as part of the 
 * function call, or the default may be set with the setCamSel() call.
 */
int FrEagleStereoCam::getCamCSR(int offset, int sel) {
  if(!camPresent) {
    printf("EagleStereoCam: getCamCSR ignored: no camera present.\n");
    return 0; // There's no way to really report the error--it should be an exception
  }
  dev->setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 0);
  int selCam = (sel==-2) ? selectedCam : sel;

  // FIXME: Attempted reimplementation of I2C interface is broken in edev
  /*
  int device = 0xB8;
  if(selCam==0) device = 0xB0;
  int data;
  if(dev->readWordI2C(offset, &data, device)) {
    // The I2C transfer failed! either retry or clear the camera present flag.
    printf("EagleStereoCam: I2C read transaction to camera failed!\n");
    printf("EagleStereoCam: Disabling further transactions.");
    camPresent = false;
  }
  return data;
  */
  return dev->getI2CCSR(offset, selCam);
}


/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * Low-Level Functions
 ******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/**
 * resetCameras() shuts down the camera sensors. LVDS is powered down and all
 * registers are reset to power-on values. The chips are reset.
 *
 * Returns error (non-zero) if any I2C transactions fail or if any error checks
 * after reset return errors. 
 */
int FrEagleStereoCam::resetCameras() {
  // Disable the VRX and Serdes, write reset values into all camera registers.
  dev->setCSR(EAGLE_CSR_VRX_EN, 0);
  dev->setCSR(EAGLE_CSR_VRX_RPWDN, 0);
  dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 0);
  //dev->setCSR(EAGLE_CSR_AEGC_SKIP_FRAMES, 0);
  setCamSel(-1);
  setCamCSR(MT9V022_LVDS_MASTER_CTRL, 0x0002); // lvds_master_ctrl
  setCamCSR(MT9V022_LVDS_SHIFT_CLK_CTRL, 0x0010); // lvds_shift_clk_ctrl
  setCamCSR(MT9V022_LVDS_DATA_CTRL, 0x0010); // lvds_data_ctrl
  setCamCSR(MT9V022_CHIP_CONTROL, 0x0388); // chip control
  setCamCSR(MT9V022_RESET, 0x0003);
  resetRegValues();
  //usleep(1000000);
  //printf("FrEagleStereoCam::resetCameras: Deserializer lock value is %d\n", dev->getCSR(EAGLE_CSR_VRX_LOCK));
  return 0;
}


/**
 * Sets the internal copies that this class keeps of various image sensor CSRs
 * back to the default values that the image sensors start with after power up.
 * There's something a little bit wrong with the internal copies this class
 * keeps of csr values because it doesn't keep separate values for both the
 * master and slave sensors.
 */
void FrEagleStereoCam::resetRegValues() {
  lvds_sensor_chip_control.val	   = 0x0388;
  lvds_sensor_reset.val		   = 0x0000;
  lvds_sensor_pixel_mode.val	   = 0x0011;
  lvds_sensor_adc_resolution.val   = 0x0002;
  lvds_sensor_row_noise_cor_1.val  = 0x0034;
  lvds_sensor_row_noise_cor_2.val  = 0x02f7;
  lvds_sensor_test_pattern.val     = 0x0000;
  lvds_sensor_agc_aec_desired_bin.val = 0x003a;
  lvds_sensor_agc_aec_enable.val   = 0x0003;
  lvds_sensor_master_control.val   = 0x0002;
  lvds_sensor_sclk_control.val     = 0x0010;
  lvds_sensor_data_control.val     = 0x0010;
  lvds_sensor_internal_sync.val    = 0x0000;
  lvds_sensor_stereo_err_ctrl.val  = 0x0000;
  lvds_sensor_stereo_err_flag.val  = 0x0000;
  lvds_sensor_analog_control.val   = 0x0840;
  lvds_sensor_reserved_core_10.val = 0x002d;
  lvds_sensor_reserved_core_15.val = 0x0e32;
  lvds_sensor_reserved_core_20.val = 0x0011;
  lvds_sensor_reserved_core_21.val = 0x0020;

}


/**
 * Reads and returns the identification numbers from each of the image sensor
 * chips in the stereo camera. Each sensor has a 64 bit identifier. The id for
 * each chip fits in two ints. 4 ints holds both the ids.
 * The result is that camSerNum[3] holds slave fuseID 63:32, camSerNum[2] holds
 * slave fuseID 31:0, camSerNum[1] holds master fuseID 63:32, and camSerNum[0]
 * holds master fuseID 31:0.
 *
 * The 64 bit fuseID numbers are broken out as lotID 0:22, wafer 23:28, diex
 * 29:35, diey 36:42, rev 59:53. The full number is guarenteed to be unique 
 * across all parts of this type.
 */
int *FrEagleStereoCam::getCamSerNum() {
  int w0 = getCamCSR(MT9V022_FUSE_WORD_1, 1);
  int w1 = getCamCSR(MT9V022_FUSE_WORD_2, 1);
  int w2 = getCamCSR(MT9V022_FUSE_WORD_3, 1);
  int w3 = getCamCSR(MT9V022_FUSE_WORD_4, 1);
  camSerNum[3] = (w3<<16) | w2;
  camSerNum[2] = (w1<<16) | w0;
  w0 = getCamCSR(MT9V022_FUSE_WORD_1, 0);
  w1 = getCamCSR(MT9V022_FUSE_WORD_2, 0);
  w2 = getCamCSR(MT9V022_FUSE_WORD_3, 0);
  w3 = getCamCSR(MT9V022_FUSE_WORD_4, 0);
  camSerNum[1] = (w3<<16) | w2;
  camSerNum[0] = (w1<<16) | w0;

  //printf("Master image sensor sernum is 0x%8.8x%8.8x\n", camSerNum[1], camSerNum[0]);
  //printf("Slave image sensor sernum is 0x%8.8x%8.8x\n", camSerNum[3], camSerNum[2]);
  return camSerNum;
}

/**
 * Assumes 37.037037ns pixel clock
 * Always use 33.333333 us/line for 60fps derivatives, 40us/line for 50fps derivatives
 * According to the spec, there's an extra 4 pixel clocks in there, so I'm always off
 * by a little bit--shouldn't be a big issue.
 */
int FrEagleStereoCam::setFrameRate(int fps) {
  // I'm not sure whether it's better to set frame rate in frames per second or
  // as the number of seconds per frame. Maybe I can support both.
  // 60 fps == 16.66 ms/frame == 0.01666 sec/frame == 450,000 pixel clock/frame == 900x500
  // 50 fps == 20 ms/frame == 540,000 == 1080*500
  // 30 fps == 33.33 ms/frame == 60fps with 1/2 frame skip or 1000 lines instead of 500
  // 25 fps == 40 ms/frame == 50fps with 1/2 frame skip or 1000 lines
  // 20 fps == 50 ms/frame == 60fps with 2/3 frame skip or 1500 lines
  // 15 fps == 66.66 ms/frame == 60fps with 3/4 frame skip or 2000 lines
  // 10 fps == 100 ms/frame == 60fps with 5/6 frame skip OR 50fps with 4/5 skip
  // 5, 2, 1 fps == 200, 500, 1000 ms/frame won't work without wider skip range
  // for now pixel clock is 37.037037 ns and I'll just make that a constant.
  // These functions accept requested rate and return actual frame rate, -1 on error.
  // Maybe also set rate to 0 to stop camera?
  //float setFrameRate(float sec); ???
  dev->setCSR(EAGLE_CSR_VRX_FRAME_SKIP_N, 0);
  dev->setCSR(EAGLE_CSR_VRX_FRAME_SKIP_M, 2);
  setCamCSR(MT9V022_WINDOW_HEIGHT, 480);
  setCamCSR(MT9V022_WINDOW_WIDTH, 752);
  switch(fps) {
  case 60: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 500-480, -1);
    return 60;
  }
  case 50: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 1080-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 500-480, -1);
    return 50;
  }
  case 30: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 1000-480, -1);
    return 30;
  }
  case 25: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 1080-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 1000-480, -1);
    return 25;
  }
  case 20: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 1500-480, -1);
    return 20;
  }
  case 15: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 2000-480, -1);
    return 15;
  }
  case 10: {
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 3000-480, -1);
    return 10;
  }
  case 5: {
    dev->setCSR(EAGLE_CSR_VRX_FRAME_SKIP_N, 1);
    dev->setCSR(EAGLE_CSR_VRX_FRAME_SKIP_M, 2);
    setCamCSR(MT9V022_HORIZONTAL_BLANKING, 900-752, -1);
    setCamCSR(MT9V022_VERTICAL_BLANKING, 3000-480, -1);
    return 5;
  }
  default: {
    return -1;
  }
  };
}

/* Test data: The programmable test data and gray shade test data pattern should
 * be easily accessable from this level. That data could really help in 
 * diagnosing camera interface issues and checking whether there are camera 
 * interface freezes, etc.
 */
// FIXME: add functions to turn on test data instead of normal video


/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * Exposure and Image Controls
 ******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/**
 * setAegcMode
 * 0 manual
 * 1 image sensor controlled aegc (unlocked)
 * 2 fpga controlled aegc (locked)
 * else manual
 */
void FrEagleStereoCam::setAegcMode(int m) {
  if(m==1) {
    lvds_sensor_agc_aec_enable.bit.aec_enable = 1;
    lvds_sensor_agc_aec_enable.bit.agc_enable = 1;
    setCamCSR( I2C_DEVICE_AGC_AEC_ENABLE, lvds_sensor_agc_aec_enable.val );
    dev->setCSR(EAGLE_CSR_AEGC_SKIP_FRAMES, 1);
    dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 0);
  } else if(m==2) {
    lvds_sensor_agc_aec_enable.bit.aec_enable = 0;
    lvds_sensor_agc_aec_enable.bit.agc_enable = 0;
    setCamCSR( I2C_DEVICE_AGC_AEC_ENABLE, lvds_sensor_agc_aec_enable.val );
    dev->setCSR(EAGLE_CSR_AEGC_SKIP_FRAMES, 1);
    dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 1);
  } else {
    lvds_sensor_agc_aec_enable.bit.aec_enable = 0;
    lvds_sensor_agc_aec_enable.bit.agc_enable = 0;
    setCamCSR( I2C_DEVICE_AGC_AEC_ENABLE, lvds_sensor_agc_aec_enable.val );
    dev->setCSR(EAGLE_CSR_AEGC_SKIP_FRAMES, 1);
    dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 0);
  }
  aegcMode = m;
}

void FrEagleStereoCam::setAEGCDesiredBin(int b) {
  // set reg 0xA5 (165) in both sensors to b, which must be 1-64 inclusive
  // turn on Eagle's AEGC block (no effect if alreay on, right?)
  // sensors are expected to always be in manual exposure/gain mode individually
  // so that part doesn't need to be changed.
  // The AEGC block just reads desired bin and actual bin of both sensors (?) repeatedly
  // and sets the manual exposure and gain repeatedly

  // Maybe there should be one function to set lockstep desired bin with the 
  // AEGC block in Eagle and another to set automatic gain and exposure in each
  // individual image sensor. That could help with comparisons.
  if(b<1 || b>64) return;
  setCamCSR(MT9V022_AEC_AGC_DESIRED_BIN, b, -1);
  // Reenable fpga aegc if we're in that mode and it was turned of due to directly
  // controlling exposure or gain in the gui. Really, this is only a function of
  // the gui and should be completely handled there, but it's a hack here for now.
  if(aegcMode==2) {
    dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 1);
  }
}

void FrEagleStereoCam::setManualGain(int g) {
  // turn off Eagle's AEGC block
  // set reg 0x35 (53) to g, which must be between 16 and 64 inclusive
  // it might be nice to also allow setting digital gain
  //if(g<16 || g>64) return;
  //dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 0);
  //setCamCSR(MT9V022_ANALOG_GAIN, g, -1);
  setAGain(g);
}

void FrEagleStereoCam::setManualExposure(int e) {
  // turn off Eagle's AEGC block
  // set reg 0x0b (11) to e, which must be between 1 and 480 inclusive
  // exposure would also be controlled by horizontal blanking time which controls
  // line length
  //if(e<1 || e>480) return;
  //dev->setCSR(EAGLE_CSR_AEGC_ENABLE, 0);
  //setCamCSR(MT9V022_TOTAL_SHUTTER_WIDTH, e, -1);
  setExp(e);
}

/*
 * Tiled digital gain: currently not used. Could really fix up problems with 
 * getting the right exposure when capturing the checkerboard. Could also help
 * with ignoring bright lights that wonder into the screen. Could also give
 * some extra gain in very dark scenes.
 */
void FrEagleStereoCam::initAegcRoi() {
  // Set all tiles to have 0 AEGC sample weight except the center one
  for(int i=0; i<25; i++) {
    setCamCSR(MT9V022_TILED_DIGITAL_GAIN_X0Y0+i, 0x04, -1);
  }
  setCamCSR(MT9V022_TILED_DIGITAL_GAIN_X2Y2, 0xF4, -1);

  // Set tile coordinates so the center tile is the whole screen
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_1X, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_2X, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_3X, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_4X, 752, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_5X, 752, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_6X, 752, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_1Y, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_2Y, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_3Y, 0, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_4Y, 480, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_5Y, 480, -1);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_6Y, 480, -1);
}

/**
 *
 */
void FrEagleStereoCam::setAegcRoi(int x1, int y1, int x2, int y2, int sel) {
  if(x1<0 || x1>752) x1=0;
  if(x2<0 || x2>752) x2=752;
  if(y1<0 || y1>480) y1=0;
  if(y2<0 || y2>480) y2=480;
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_3X, x1, sel);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_4X, x2, sel);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_3Y, y1, sel);
  setCamCSR(MT9V022_DIGITAL_TILE_COORD_4Y, y2, sel);
}

/**
 *
 */
void FrEagleStereoCam::setExp(int e, int s) {
  if(e<1 || e>480) return;
  if(s<-1 || s>1) return;
  setCamCSR(MT9V022_TOTAL_SHUTTER_WIDTH, e, s);
}

/**
 *
 */
int FrEagleStereoCam::getExp(int s) {
  if(s<-1 || s>1) return -1;
  return getCamCSR(MT9V022_TOTAL_SHUTTER_WIDTH, s);

}

/**
 *
 */
void FrEagleStereoCam::setAGain(int g, int s) {
  if(s<-1 || s>1) return;
  if(g<16 || g>64) return;
  setCamCSR(MT9V022_ANALOG_GAIN, g, s);
}

/**
 *
 */
int FrEagleStereoCam::getAGain(int s) {
  if(s<-1 || s>1) return -1;
  return getCamCSR(MT9V022_ANALOG_GAIN, s);
}

/**
 *
 */
void FrEagleStereoCam::initDGain() {
}

/**
 *
 */
void FrEagleStereoCam::setDGain(int g, int s) {
  if(s<-1 || s>1) return;
}

/**
 *
 */
int FrEagleStereoCam::getDGain(int s) {
  if(s<-1 || s>1) return -1;
  return 0;
}

/**
 *
 */

/**
 *
 */


/* Non linear mode: this could really help with outside scenes. Not getting
 * complaints now, but I would think this could really help. There's lots of
 * flexibility in how this works, but I'm not sure how much is needed.
 */


/******************************************************************************/
/******************************************************************************/
/*******************************************************************************
 * High-Level Operations
 ******************************************************************************/
/******************************************************************************/
/******************************************************************************/


/**
 * initCamera() performs top level camera initialization
 */
int FrEagleStereoCam::initCamera() {
  printf("FrEagleStereoCam: initCamera()\n");
  resetCameras();
  // FIXME: We should zero out image memory here!!
  //printf("FrEagleDevice: uncalibrated base addresses\n");    
  dev->setCSR(EAGLE_CSR_VRX_BASE_ADDR_CH0, 0);
  dev->setCSR(EAGLE_CSR_VRX_BASE_ADDR_CH1, 4);

  bringupCameras();

  // This retry loop is a little kludgy--there should be better interaction with
  // higher levels so everything doesn't just hang while the camera tries to
  // start over and over. Also, it shouldn't sit in this loop if the camera isn't
  // present at all; i.e. if we aren't getting i2c acks on any access.
  int timeout = 0;
  while(startCameras()) {
    usleep(500000);
    timeout++;
    if(timeout>10) {
      printf("Unable to start camera after 10 retries.\n");
      exit(1);
    }
    resetCameras();
    bringupCameras();
  }
  setupCameras();
  setAegcMode(aegcMode);
  setFrameRate(30);

  return 0;
}

/**
 * initCameras() initializes the cameras to start sending LVDS information in
 * stereoscopic mode. It consists of programming lots of camera registers and
 * checking the stereo error flag to verify no inter-sensor skew.
 *
 * Returns error (non-zero) if any I2C transactions fail or if LVDS can't be
 * started without setting off the stereo_error_flag.
 * FIXME: move stereo error detection flag to this function from startCameras()
 * FIXME: try making this function follow the procedure in the spec more closely
 * FIXME: adjust blanking so camera runs at exactly 60Hz or 50Hz here
 * FIXME: Other image adjustments should maybe also occur here instead of after 
 *        the data check--consider that because then the first images out after
 *        bringup would be usable.
 */
int FrEagleStereoCam::bringupCameras() {
  // Assume the cameras are in reset state on entry
  // Follow startup procedure on page 64 in MT9V022 spec
  //printf("Enable master LVDS bypass clk	: (R177[0]=1)\n");
  lvds_sensor_master_control.bit.pll_bypass = 1;
  setCamCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 0);

  //printf("Disable camera parallel output	: (R7[7]=0) \n");
  //printf("Put camera in stereoscopy mode	: (R7[5]=0) \n");
  lvds_sensor_chip_control.bit.parallel_output_enable = 0;   
  lvds_sensor_chip_control.bit.stereo_mode = 1;
  setCamCSR(I2C_DEVICE_CHIP_CONTROL, lvds_sensor_chip_control.val);

  //printf("Enable LVDS serial data out driver	: (R179[4]=0) \n");
  lvds_sensor_data_control.bit.power_down = 0; // enable master
  setCamCSR(I2C_DEVICE_DATA_CONTROL_ADDR, lvds_sensor_data_control.val, 0);
  lvds_sensor_data_control.bit.power_down = 1; // diable slave
  setCamCSR(I2C_DEVICE_DATA_CONTROL_ADDR, lvds_sensor_data_control.val, 1);

  //printf("Enable LVDS shift clk out driver	: (R178[4]=0) \n");
  lvds_sensor_sclk_control.bit.power_down = 1; // disable master
  setCamCSR(I2C_DEVICE_SCLK_CONTROL_ADDR, lvds_sensor_sclk_control.val, 0);
  lvds_sensor_sclk_control.bit.power_down = 0; // enable slave
  setCamCSR(I2C_DEVICE_SCLK_CONTROL_ADDR, lvds_sensor_sclk_control.val, 1);

  //printf("De-assert LVDS powerdown		: (R177[1]=0)\n");
  lvds_sensor_master_control.bit.pll_bypass = 1;
  lvds_sensor_master_control.bit.lvds_power_down = 0;
  setCamCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 0);
  lvds_sensor_master_control.bit.pll_bypass = 0;
  lvds_sensor_master_control.bit.lvds_power_down = 0;
  setCamCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 1);
    
  //printf("Issue a lvds sensor soft reset	: (R12[0]=1 followed by R12[0]=0)\n");
  lvds_sensor_reset.bit.soft_reset = 1;
  lvds_sensor_reset.bit.auto_block_soft_reset = 1;
  setCamCSR(I2C_DEVICE_RESET_ADDR, lvds_sensor_reset.val);
  lvds_sensor_reset.bit.soft_reset = 0;
  lvds_sensor_reset.bit.auto_block_soft_reset = 0;
  setCamCSR(I2C_DEVICE_RESET_ADDR, lvds_sensor_reset.val);

  //printf("Set stereoscopic slave		: (R7[6]=1) \n");
  lvds_sensor_chip_control.bit.stereo_master_slave_mode = 1;
  setCamCSR(I2C_DEVICE_CHIP_CONTROL, lvds_sensor_chip_control.val);

  //printf("Enable lvds sensor sync patterns	: (R181[0]=1)\n");
  lvds_sensor_internal_sync.bit.sync_enable = 1;
  setCamCSR(I2C_DEVICE_INTERNAL_SYNC_ADDR, lvds_sensor_internal_sync.val);

  //printf("Enable master's stereo error detection mechanisim\n");
  //lvds_sensor_stereo_err_ctrl.bit.enable_error_detect = 1;
  //lvds_sensor_stereo_err_ctrl.bit.enable_stick_flag = 1;
  setCamCSR(I2C_DEVICE_STEREO_ERR_CTRL_ADDR, 0x0007, 0);
  //setCamCSR(I2C_DEVICE_STEREO_ERR_CTRL_ADDR, lvds_sensor_stereo_err_ctrl.val, 0);

  //printf("Check master's stereo error flag:");
  lvds_sensor_stereo_err_flag.val  = getCamCSR(I2C_DEVICE_STEREO_ERR_FLAG_ADDR, 0); 
  if (lvds_sensor_stereo_err_flag.bit.error_flag == 1) {
    printf("ERROR: master's stereo error flag is set\n");
    lvds_sensor_lvds_data_output.val = getCamCSR(I2C_DEVICE_LVDS_DATA_OUTPUT_ADDR, 0);
    printf("LVDS Data Output is: %x\n", lvds_sensor_lvds_data_output.bit.combo_reg);
    exit(1);
    // FIXME: should be exception
  } else {
    //printf("PASS\n");
  }

  return 0;
}


/**
 * startCameras() actually starts the video stream into Eagle.
 * This includes setting up the deserializer and vrx and verifying that the 
 * incoming stream is valid.
 *
 * Returns error (non-zero) if any I2C transactions fail or if a valid incoming
 * video stream can't be started.
 * FIXME: add test data and a data check to prevent one camera coming up black
 * FIXME: add a retry if the deserializer doesn't lock for some reason
 */
int FrEagleStereoCam::startCameras() {
  // Assume cameras are sending sync packets but serdes and vrx are disabled on
  // entry to this routine.
  //printf("Poweron deserializer\n");
  dev->setCSR(EAGLE_CSR_VRX_RPWDN, 1);

  //printf("FrEagleStereoCam::startCameras: Deserializer lock value before waiting is %d\n", dev->getCSR(EAGLE_CSR_VRX_LOCK));
  // Wait up to 200ms for deserializer to lock on camera sync packets
  if(dev->waitCSR(EAGLE_CSR_VRX_LOCK, 0, 1, 200)) {
    printf("FrEagleStereoCam::startCameras(): Deserializer failed to lock, check camera connections.\n");
    printf("FrEagleStereoCam::startCameras: Deserializer lock value is %d\n", dev->getCSR(EAGLE_CSR_VRX_LOCK));
    return 1;
  }
  //printf("FrEagleStereoCam::startCameras: Deserializer lock value after waiting is %d\n", dev->getCSR(EAGLE_CSR_VRX_LOCK));

  //printf("Enable camera RX unit\n");
  dev->setCSR(EAGLE_CSR_VRX_EN, 1);

  // NOTE: we should be getting black frams from both sensors now!
  // perhaps I should double check that while debugging
  usleep(50000);
  if(testBlackFrame(0)) printf("FrEagleStereoCam: Black Frame at src 0 before turning off sync packets\n");
  if(testBlackFrame(4)) printf("FrEagleStereoCam: Black Frame at src 4 before turning off sync packets\n");

  //printf("disable lvds sensor sync patterns	: (R181[0]=0)\n");
  lvds_sensor_internal_sync.bit.sync_enable = 0;
  setCamCSR(I2C_DEVICE_INTERNAL_SYNC_ADDR, lvds_sensor_internal_sync.val);    

  // VERIFY that the frame count is incrementing
  //printf("Check frame_count		: ");
  int current_frame = dev->getCSR(EAGLE_CSR_VRX_FRAME_CNT);
  int last_frame = current_frame;
  int i;
  int timeout;
  for (i = 0; i < 10; i++) {
    timeout = 0;
    while(current_frame==last_frame) {
      usleep(1000);
      current_frame = dev->getCSR(EAGLE_CSR_VRX_FRAME_CNT);
      timeout ++;
      if(timeout > 100) {
	printf("FrEagleStereoCam::startCameras: Timeout waiting for frame while checking frame count!\n");
	printf("FrEagleStereoCam::startCameras: Deserializer lock value is %d\n", dev->getCSR(EAGLE_CSR_VRX_LOCK));
	printf("FrEagleStereoCam::startCameras: VRX_ERROR is %d\n", dev->getCSR(EAGLE_CSR_VRX_ERROR));
	printf("FrEagleStereoCam::startCameras: VRX_ERROR_STATUS is %d\n", dev->getCSR(EAGLE_CSR_VRX_ERROR_STATUS));
	printf("FrEagleStereoCam::startCameras: VRX_FRAME_CNT is %d\n", dev->getCSR(EAGLE_CSR_VRX_FRAME_CNT));
	return 2;
      }
    }
    if(current_frame != (last_frame+1)%4) {
      printf("missed frame %d, last_frame=%d, current_frame=%d\n", i, last_frame, current_frame);
      break;
    }
    last_frame = current_frame;
  }
  //printf("PASS\n");

  // Check for any error codes set in the VRX block in the FPGA
  //printf("Check VRX error		: ");
  if (dev->getCSR(EAGLE_CSR_VRX_ERROR) == 1) {
    printf("FrEagleStereoCam::startCameras: ERROR: VRX Error detected, error_status is %d\n", dev->getCSR(EAGLE_CSR_VRX_ERROR_STATUS));
    //return 3;
  } else {
    printf("error check PASS\n");
  }

  // FIXME: Check for black frames
  if(testBlackFrame(0)) {
    printf("FrEagleStereoCam: Black Frame at src 0\n");
    return 4;
  }
  if(testBlackFrame(4)) {
    printf("FrEagleStereoCam: Black Frame at src 4\n");
    return 5;
  }

  return 0;
}

/**
 * 
 */
int FrEagleStereoCam::testBlackFrame(int src) {
  uchar *buff = (uchar*)malloc(368640);
  dev->grabFrame(0, src);
  dev->retrieveFrame(0, buff);
  for(int y=0;y<480;y++) {
    uchar *line = buff + (y*768);
    for(int x=0;x<752;x++) {
      if(line[x]!=0) {
	free(buff);
	return 0;
      }
    }
  }
  free(buff);
  return 1;
}

/**
 * setupCameras() sets image properties for the cameras.
 * This includes setting image size if a smaller size has been chosen, setting
 * auto gain and exposure, setting timing for 50Hz or 60Hz operation, and
 * setting image properties like no IR filter mode.
 *
 * Returns error (non-zero) if any I2C transactions fail.
 */
int FrEagleStereoCam::setupCameras() {
  // It doesn't matter whether cameras are running or not when this routine is
  // entered. Just write control registers as required.

  // Image adjustments
  //printf("Adjust for no-IR filter                   (R112[3:0]=0x2)\n");
  lvds_sensor_row_noise_cor_1.bit.number_dark_pixels = 2;
  setCamCSR( I2C_DEVICE_ROW_NOISE_COR_1_ADDR, lvds_sensor_row_noise_cor_1.val );
  
  //printf("Adjust for no-IR filter                   (R115[9:0]=0x307)\n");
  lvds_sensor_row_noise_cor_2.bit.dark_column_start_addr = 0x307;
  setCamCSR( I2C_DEVICE_ROW_NOISE_COR_2_ADDR, lvds_sensor_row_noise_cor_2.val );  

  //printf("fix fpn                                   (R16[6:0]=0x40)\n");
  lvds_sensor_reserved_core_10.bit.fpn_fix = 0x40;
  setCamCSR( I2C_DEVICE_RESERVED_10_ADDR, lvds_sensor_reserved_core_10.val );

  //printf("fix fpn                                   (R21[15:8]=0x7F)\n");
  lvds_sensor_reserved_core_15.bit.fpn_fix = 0x7F; // if not hi-dy
  //lvds_sensor_reserved_core_15.bit.fpn_fix = 0x28;     
  setCamCSR( I2C_DEVICE_RESERVED_15_ADDR, lvds_sensor_reserved_core_15.val );

  //printf("fix fpn                                   (R32[8:6]=0x7)\n");
  lvds_sensor_reserved_core_20.bit.fpn_fix = 7;
  setCamCSR( I2C_DEVICE_RESERVED_20_ADDR, lvds_sensor_reserved_core_20.val );

  //printf("disable agc/aec\n");
  lvds_sensor_agc_aec_enable.bit.aec_enable = 0;
  lvds_sensor_agc_aec_enable.bit.agc_enable = 0;
  setCamCSR( I2C_DEVICE_AGC_AEC_ENABLE, lvds_sensor_agc_aec_enable.val );

  // Set default brightness target
  lvds_sensor_agc_aec_desired_bin.bit.desired_bin = 35;
  setCamCSR(I2C_DEVICE_AGC_AEC_DESIRED_BIN_ADDR, lvds_sensor_agc_aec_desired_bin.val);

  /*printf("enable hi-dynamic range                   (R15[6]=1)\n");
  lvds_sensor_reserved_core_15.bit.fpn_fix = 0x28;     
  setCamCSR( I2C_DEVICE_RESERVED_15_ADDR, lvds_sensor_reserved_core_15.val );
  */
  lvds_sensor_pixel_mode.bit.hidy_enable = 1;
  setCamCSR ( I2C_DEVICE_PIXEL_MODE_ADDR, lvds_sensor_pixel_mode.val );

  return 0;
}

/**
 * This is the original camera bringup routine that has been in use for well
 * over a year. It works and is proven, and is almost exactly the same code that
 * in in the fr3 V4L driver and windows driver. It is, however, one monolithic
 * routine that doesn't support much diagnostic information or feedback for use
 * with questionable or new hardware. 
 */
void FrEagleStereoCam::oldBringupCameras() {
  // Bringup CAMERA
  printf("Disable camera RX unit\n");
  dev->setCSR(EAGLE_CSR_VRX_EN, 0); 

  printf("Powerdown deserializer\n");
  dev->setCSR(EAGLE_CSR_VRX_RPWDN, 0);

  // FIXME: We don't need to actually read these. We can just set them to known chip defaults.
  printf("Read register defaults\n");
  lvds_sensor_chip_control.val	   = dev->getI2CCSR(I2C_DEVICE_CHIP_CONTROL, 0); 
  lvds_sensor_reset.val		   = dev->getI2CCSR(I2C_DEVICE_RESET_ADDR	, 0); 
  lvds_sensor_pixel_mode.val	   = dev->getI2CCSR(I2C_DEVICE_PIXEL_MODE_ADDR, 0); 
  lvds_sensor_adc_resolution.val   = dev->getI2CCSR(I2C_DEVICE_ADC_RESOLUTION_ADDR, 0); 
  lvds_sensor_row_noise_cor_1.val  = dev->getI2CCSR(I2C_DEVICE_ROW_NOISE_COR_1_ADDR, 0); 
  lvds_sensor_row_noise_cor_2.val  = dev->getI2CCSR(I2C_DEVICE_ROW_NOISE_COR_2_ADDR, 0); 
  lvds_sensor_test_pattern.val     = dev->getI2CCSR(I2C_DEVICE_TEST_PATTERN_ADDR, 0); 
  lvds_sensor_agc_aec_desired_bin.val = dev->getI2CCSR(I2C_DEVICE_AGC_AEC_DESIRED_BIN_ADDR, 0); 
  lvds_sensor_agc_aec_enable.val   = dev->getI2CCSR(I2C_DEVICE_AGC_AEC_ENABLE, 0); 
  lvds_sensor_master_control.val   = dev->getI2CCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, 0); 
  lvds_sensor_sclk_control.val     = dev->getI2CCSR(I2C_DEVICE_SCLK_CONTROL_ADDR, 0); 
  lvds_sensor_data_control.val     = dev->getI2CCSR(I2C_DEVICE_DATA_CONTROL_ADDR, 0); 
  lvds_sensor_internal_sync.val    = dev->getI2CCSR(I2C_DEVICE_INTERNAL_SYNC_ADDR, 0); 
  lvds_sensor_stereo_err_ctrl.val  = dev->getI2CCSR(I2C_DEVICE_STEREO_ERR_CTRL_ADDR, 0); 
  lvds_sensor_stereo_err_flag.val  = dev->getI2CCSR(I2C_DEVICE_STEREO_ERR_FLAG_ADDR, 0); 
  lvds_sensor_analog_control.val   = dev->getI2CCSR(I2C_DEVICE_ANALOG_CONTROL_ADDR, 0); 
  lvds_sensor_reserved_core_10.val = dev->getI2CCSR(I2C_DEVICE_RESERVED_10_ADDR, 0); 
  lvds_sensor_reserved_core_15.val = dev->getI2CCSR(I2C_DEVICE_RESERVED_15_ADDR, 0); 
  lvds_sensor_reserved_core_20.val = dev->getI2CCSR(I2C_DEVICE_RESERVED_20_ADDR, 0); 
  lvds_sensor_reserved_core_21.val = dev->getI2CCSR(I2C_DEVICE_RESERVED_21_ADDR, 0); 

  printf("Disable stereo error detect\n");
  lvds_sensor_stereo_err_ctrl.bit.enable_error_detect = 0;
  lvds_sensor_stereo_err_ctrl.bit.enable_stick_flag = 0;
  lvds_sensor_stereo_err_ctrl.bit.clear_error_flag = 1;
  dev->setI2CCSR(I2C_DEVICE_STEREO_ERR_CTRL_ADDR, lvds_sensor_stereo_err_ctrl.val, 0);


  printf("Enable master LVDS bypass clk	: (R177[0]=1)\n");
  lvds_sensor_master_control.bit.pll_bypass = 1;
  dev->setI2CCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 0);

  printf("Disable camera parallel output	: (R7[7]=0) \n");
  printf("Put camera in stereoscopy mode	: (R7[5]=0) \n");
  lvds_sensor_chip_control.bit.parallel_output_enable = 0;   
  lvds_sensor_chip_control.bit.stereo_mode = 1;
  dev->setI2CCSR(I2C_DEVICE_CHIP_CONTROL, lvds_sensor_chip_control.val);

  printf("Enable LVDS serial data out driver	: (R179[4]=0) \n");
  lvds_sensor_data_control.bit.power_down = 0; // enable master
  dev->setI2CCSR(I2C_DEVICE_DATA_CONTROL_ADDR, lvds_sensor_data_control.val, 0);
  lvds_sensor_data_control.bit.power_down = 1; // diable slave
  dev->setI2CCSR(I2C_DEVICE_DATA_CONTROL_ADDR, lvds_sensor_data_control.val, 1);

  printf("Enable LVDS shift clk out driver	: (R178[4]=0) \n");
  lvds_sensor_sclk_control.bit.power_down = 1; // disable master
  dev->setI2CCSR(I2C_DEVICE_SCLK_CONTROL_ADDR, lvds_sensor_sclk_control.val, 0);
  lvds_sensor_sclk_control.bit.power_down = 0; // enable slave
  dev->setI2CCSR(I2C_DEVICE_SCLK_CONTROL_ADDR, lvds_sensor_sclk_control.val, 1);

  printf("De-assert LVDS powerdown		: (R177[1]=0)\n");
  lvds_sensor_master_control.bit.pll_bypass = 1;
  lvds_sensor_master_control.bit.lvds_power_down = 0;
  dev->setI2CCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 0);
  lvds_sensor_master_control.bit.pll_bypass = 0;
  lvds_sensor_master_control.bit.lvds_power_down = 0;
  dev->setI2CCSR(I2C_DEVICE_MASTER_CONTROL_ADDR, lvds_sensor_master_control.val, 1);
    
  printf("Issue a lvds sensor soft reset	: (R12[0]=1 followed by R12[0]=0)\n");
  lvds_sensor_reset.bit.soft_reset = 1;
  lvds_sensor_reset.bit.auto_block_soft_reset = 1;
  dev->setI2CCSR(I2C_DEVICE_RESET_ADDR, lvds_sensor_reset.val);
  lvds_sensor_reset.bit.soft_reset = 0;
  lvds_sensor_reset.bit.auto_block_soft_reset = 0;
  dev->setI2CCSR(I2C_DEVICE_RESET_ADDR, lvds_sensor_reset.val);

  printf("Set stereoscopic slave		: (R7[6]=1) \n");
  lvds_sensor_chip_control.bit.stereo_master_slave_mode = 1;
  dev->setI2CCSR(I2C_DEVICE_CHIP_CONTROL, lvds_sensor_chip_control.val);

  printf("Enable lvds sensor sync patterns	: (R181[0]=1)\n");
  lvds_sensor_internal_sync.bit.sync_enable = 1;
  dev->setI2CCSR(I2C_DEVICE_INTERNAL_SYNC_ADDR, lvds_sensor_internal_sync.val);

  printf("Poweron deserializer\n");
  dev->setCSR(EAGLE_CSR_VRX_RPWDN, 1);
  //msync(csrBase, (1024*1024), MS_SYNC); // FIXME: was this necessar?
  printf("Give time for sync to propogate\n");
  usleep(10000);
  usleep(10000);
  usleep(10000);
  usleep(10000);

  printf("Check for deserializer lock		: ");
  if ( dev->getCSR(EAGLE_CSR_VRX_LOCK) != 0 ) {
    // FIXME: should be an exception
    printf("FAIL\n");
    exit(1);
  } else {
    printf("PASS\n");
  }

  printf("Enable camera RX unit\n");
  dev->setCSR(EAGLE_CSR_VRX_EN, 1);

  //sleep(1);
  //msync(csrBase, (1024*1024), MS_SYNC); // FIXME:

  printf("disable lvds sensor sync patterns	: (R181[0]=0)\n");
  lvds_sensor_internal_sync.bit.sync_enable = 0;
  dev->setI2CCSR(I2C_DEVICE_INTERNAL_SYNC_ADDR, lvds_sensor_internal_sync.val);    

  printf("Check frame_count		: ");
  int current_frame = dev->getCSR(EAGLE_CSR_VRX_FRAME_CNT);
  int last_frame = current_frame;
  int i;
  int timeout;
  for (i = 0; i < 10; i++) {
    timeout = 0;
    while(current_frame==last_frame) {
      usleep(1000);
      current_frame = dev->getCSR(EAGLE_CSR_VRX_FRAME_CNT);
      timeout ++;
      if(timeout > 100) {
	printf("Timeout waiting for frame while checking frame count!\n");
	break;
      }
    }

    if(current_frame != (last_frame+1)%4) {
      printf("missed frame %d, last_frame=%d, current_frame=%d\n", i, last_frame, current_frame);
      break;
    }
    last_frame = current_frame;
  }
  if (i != 10) {
    printf("FAIL\n");
    exit(1);
    // FIXME: should be an exception
  } else {
    printf("PASS\n");
  }

  printf("Check VRX error		: ");
  if (dev->getCSR(EAGLE_CSR_VRX_ERROR) == 1) {
    printf("FAIL\n");
    exit(1);
  } else {
    printf("PASS\n");
  }

  printf("Enable master's stereo error detection mechanisim\n");
  lvds_sensor_stereo_err_ctrl.bit.enable_error_detect = 1;
  lvds_sensor_stereo_err_ctrl.bit.enable_stick_flag = 1;
  dev->setI2CCSR(I2C_DEVICE_STEREO_ERR_CTRL_ADDR, lvds_sensor_stereo_err_ctrl.val, 0);
  //sleep(1); 
  //msync(csrBase, (1024*1024), MS_SYNC); // FIXME:

  printf("Check master's stereo error flag	:");
  lvds_sensor_stereo_err_flag.val  = dev->getI2CCSR(I2C_DEVICE_STEREO_ERR_FLAG_ADDR, 0); 
  if (lvds_sensor_stereo_err_flag.bit.error_flag == 1) {
    printf("FAIL\n");
    lvds_sensor_lvds_data_output.val = dev->getI2CCSR(I2C_DEVICE_LVDS_DATA_OUTPUT_ADDR, 0);
    printf("LVDS Data Output is			: %x\n", lvds_sensor_lvds_data_output.bit.combo_reg);
    exit(1);
    // FIXME: should be exception
  } else {
    printf("PASS\n");
  }


  // Image adjustments (FIXME: move to own routine)
  printf("Adjust for no-IR filter                   (R112[3:0]=0x2)\n");
  lvds_sensor_row_noise_cor_1.bit.number_dark_pixels = 2;
  dev->setI2CCSR( I2C_DEVICE_ROW_NOISE_COR_1_ADDR, lvds_sensor_row_noise_cor_1.val );
  
  printf("Adjust for no-IR filter                   (R115[9:0]=0x307)\n");
  lvds_sensor_row_noise_cor_2.bit.dark_column_start_addr = 0x307;
  dev->setI2CCSR( I2C_DEVICE_ROW_NOISE_COR_2_ADDR, lvds_sensor_row_noise_cor_2.val );  

  printf("fix fpn                                   (R16[6:0]=0x40)\n");
  lvds_sensor_reserved_core_10.bit.fpn_fix = 0x40;
  dev->setI2CCSR( I2C_DEVICE_RESERVED_10_ADDR, lvds_sensor_reserved_core_10.val );

  printf("fix fpn                                   (R21[15:8]=0x7F)\n");
  lvds_sensor_reserved_core_15.bit.fpn_fix = 0x7F; // if not hi-dy
  //lvds_sensor_reserved_core_15.bit.fpn_fix = 0x28;     
  dev->setI2CCSR( I2C_DEVICE_RESERVED_15_ADDR, lvds_sensor_reserved_core_15.val );

  printf("fix fpn                                   (R32[8:6]=0x7)\n");
  lvds_sensor_reserved_core_20.bit.fpn_fix = 7;
  dev->setI2CCSR( I2C_DEVICE_RESERVED_20_ADDR, lvds_sensor_reserved_core_20.val );

  printf("disable agc/aec\n");
  lvds_sensor_agc_aec_enable.bit.aec_enable = 0;
  lvds_sensor_agc_aec_enable.bit.agc_enable = 0;  
  dev->setI2CCSR( I2C_DEVICE_AGC_AEC_ENABLE, lvds_sensor_agc_aec_enable.val );

  // FIXME? set default bin here?
  lvds_sensor_agc_aec_desired_bin.bit.desired_bin = 35;
  dev->setI2CCSR(I2C_DEVICE_AGC_AEC_DESIRED_BIN_ADDR, lvds_sensor_agc_aec_desired_bin.val);

  /*printf("enable hi-dynamic range                   (R15[6]=1)\n");
  lvds_sensor_reserved_core_15.bit.fpn_fix = 0x28;     
  dev->setI2CCSR( I2C_DEVICE_RESERVED_15_ADDR, lvds_sensor_reserved_core_15.val );
  */
  lvds_sensor_pixel_mode.bit.hidy_enable = 0;
  dev->setI2CCSR ( I2C_DEVICE_PIXEL_MODE_ADDR, lvds_sensor_pixel_mode.val );

}



