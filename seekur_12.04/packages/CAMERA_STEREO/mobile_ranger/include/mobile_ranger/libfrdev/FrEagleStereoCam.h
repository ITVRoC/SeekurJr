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
 * Brief Description: This class encapsulates the Eagle Fr2 stereo camera. It
 * provides controls to initialize and access and shutdown the camera and high
 * high level operations to set modes and behaviors.
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
#ifndef FREAGLESTEREOCAM_H
#define FREAGLESTEREOCAM_H

#include "FrEagleDevice.h"
#include "FrEagleStereoCamRegs.h"

// Master is right and slave is left.
#define EAGLE_RIGHT_CAM 0
#define EAGLE_LEFT_CAM 1
#define EAGLE_BOTH_CAM -1

/* Since this is a stereo camera object, isn't it stupid to have cam or camera
 * in the function call names? A new proposed API:
 * init(), reset(), start(), stop(), config()
 * setCSR(), getCSR()
 * It's also weird to call this a camera object, but not allow getting frames.
 * There should be getFrame methods and/or grab/retrieve methods
 * setMode, setBrightness, setExp, setAnaGain, setDigGain, setDynamicRange
 * Major modes for the camera are probably auto exposure and manual, and there
 * might be a few options for auto exposure including different algorithms and
 * a min shutter speed to capture motion.
 * I need to figure out a good interface to the ROI processing stuff and 
 * digital gain.
 * An issue with putting the software aegc algorithm in this block is that it
 * would need to be explicitly called every frame. Perhaps the run() function
 * in the smartCam could call the AegcUpdate function in this object?
 */
class FrEagleStereoCam {
 public:
  static const int FrSensorBoth = -1;
  static const int FrSensorRight = 0;
  static const int FrSensorLeft = 1;

  // Constructor/Destructor
  FrEagleStereoCam(FrEagleDevice *d);
  ~FrEagleStereoCam();

  // Basic Camera Access
  void setCamSel(int sel); // 0=master, 1=slave, -1=broadcast
  void setCamCSR(int offset, int data, int sel=-2);
  int getCamCSR(int offset, int sel=-2);

  // Low-Level Functions
  int resetCameras(); // or clearCameras or shutdownCameras
  void resetRegValues();
  int *getCamSerNum(); // might be better to just return a int or long int and pass sensor number (?)
  int setFrameRate(int fps);

  // Exposure and Image Controls
  // setAegcMode: 0=manual, 1=sensor(unlocked), 2=fpga(locked)
  void setAegcMode(int m);
  void setAEGCDesiredBin(int b);
  void setManualGain(int g);
  void setManualExposure(int e);
  //void setLockedExpGain(int en);

  // set up both image sensors to be ready for a AEGC ROI. This includes adjusting
  // tile coordinates and sample weight for all 25 regions. The middle region (2,2)
  // will be the only one with any weight for AEGC
  void initAegcRoi();
  // set a region to be the only one considered by the AEGC algorithm
  // This will basically set regs 155, 156, 161, 162
  void setAegcRoi(int x1, int y1, int x2, int y2, int sel);

  // new functions to replace some of those below; not yet implemented!
  // What units should exposure be set in for the general case?
  void setExp(int e, int s=-1);
  int getExp(int s);
  // What units should gain be set in for the general case?
  // get/set gain sets analog first and digital when beyond analog range
  // otherwise, anaGain and digGain sets specific properties
  void setGain(int g, int s=-1);
  int getGain(int s);
  void setAGain(int g, int s=-1);
  int getAGain(int s);
  void initDGain();
  void setDGain(int g, int s=-1);
  int getDGain(int s);
  // Non-linear exposure mode, AKA high dynamic range mode, allows compression 
  // of very bright image regions into fewer bits so overexposure and blooming
  // is less likely to occur. The MT9V022 has some very specific controls for 
  // this feature, but I would like a generalized interface as well, perhaps
  // just selecting various choices of non-linear curve.
  // The full interface for this sensor should provide a safe way to program all
  // voltages and integration times at once.
  void setNonLinearExp(int en, int s=-1);
  int getNonLinearExp(int s);

  // High-Level Operations to initialize and bringup the camera
  int initCamera();
  int bringupCameras(); // was init, starts LVDS stream--maybe swap bringup and start names?
  int startCameras(); // starts serdes and vrx and checks images and framerate--better name?
  int setupCameras(); // sets up image characteristics: size, brightness, framerate, etc.
  void oldBringupCameras();


 protected:
  int testBlackFrame(int src);

 private:
  FrEagleDevice *dev;
  bool camPresent;
  int selectedCam;
  int camSerNum[4];
  int aegcMode;

  lvds_sensor_chip_control_u     lvds_sensor_chip_control;
  lvds_sensor_shutter_width_u    lvds_sensor_shutter_width;
  lvds_sensor_reset_u            lvds_sensor_reset;
  lvds_sensor_pixel_mode_u       lvds_sensor_pixel_mode;
  lvds_sensor_adc_resolution_u   lvds_sensor_adc_resolution;
  lvds_sensor_row_noise_cor_1_u  lvds_sensor_row_noise_cor_1;
  lvds_sensor_row_noise_cor_2_u  lvds_sensor_row_noise_cor_2;
  lvds_sensor_test_pattern_u     lvds_sensor_test_pattern;
  lvds_sensor_agc_aec_desired_bin_u lvds_sensor_agc_aec_desired_bin;
  lvds_sensor_agc_aec_enable_u   lvds_sensor_agc_aec_enable;
  lvds_sensor_master_control_u   lvds_sensor_master_control;
  lvds_sensor_sclk_control_u     lvds_sensor_sclk_control;
  lvds_sensor_data_control_u     lvds_sensor_data_control;
  lvds_sensor_internal_sync_u    lvds_sensor_internal_sync;
  lvds_sensor_stereo_err_ctrl_u  lvds_sensor_stereo_err_ctrl;
  lvds_sensor_stereo_err_flag_u  lvds_sensor_stereo_err_flag;
  lvds_sensor_analog_control_u   lvds_sensor_analog_control;
  lvds_sensor_reserved_core_10_u lvds_sensor_reserved_core_10;
  lvds_sensor_reserved_core_15_u lvds_sensor_reserved_core_15;
  lvds_sensor_reserved_core_20_u lvds_sensor_reserved_core_20;
  lvds_sensor_reserved_core_21_u lvds_sensor_reserved_core_21;
  lvds_sensor_lvds_data_output_u lvds_sensor_lvds_data_output;
};

#endif

