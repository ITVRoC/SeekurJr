/****************************************************************************
 * Copyright (c) 2008 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Mon Nov 10 2008
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

#ifndef FRSTEREOSMARTCAMGUI_H
#define FRSTEREOSMARTCAMGUI_H

#include "FrEagleStereoSmartCam.h"
#include <sys/time.h>
#include "highgui.h"

// Because of the limitations of OpenCV highgui, this class must be a singleton.
// Trackbars don't pass a data pointer to their callback, so the callback must 
// access global data. Alternatively, maybe the trackbar should just be checked
// for updates once per cycle and not use the callback at all.
class FrStereoSmartCamGUI {
 public:
  // Constructors
  FrStereoSmartCamGUI(FrEagleStereoSmartCam *sscam, const char *name);
  ~FrStereoSmartCamGUI();

  // This is only needed if the stereo smart cam changes, like when I shut down
  // and restart in the demo program, or reload the firmware.
  void setStereoSmartCam(FrEagleStereoSmartCam *sscam) {ssc = sscam;}
  // The control window starts out visible, but can be shown or hidden
  void hide(int type=0);
  void show(int type=0);
  void move(int x, int y, int type=0);
  // called once per frame to update monitors, check stats, change params as 
  // necessary. Could be passed the latest keypress if desired.
  void update(int kp=0);

 protected:
  void init();
  // create the window, add the trackbars, create the image to draw on
  // This is the original control window, keep for backward compatibility
  void createControlWindow();
  void destroyControlWindow();
  void drawControlWindow(); // do all the puttext commands based on current monitor values

  // AEGC enable, settings, left and right exposure, analog gain, digital gain
  void createExposureControlWin();
  void destroyExposureControlWin();
  void redrawExposureControlWin();

  // Controls to enable the mode, enable auto-adjust and auto-adj time ratios
  // controls to adjust times and voltages for manual high dynamic range
  void createHighDynControlWin();
  void destroyHighDynControlWin();
  void redrawHighDynControlWin();

  // Mostly just display, I don't think there will be any controls
  void createHistogramWin();
  void destroyHistogramWin();
  void redrawHistogramWin();

  // set left-right check, set match qual check, set disp search range, 
  // set post-process surface check, set subpixel interp, etc.
  void createStereoControlWin();
  void destroyStereoControlWin();
  void redrawStereoControlWin();

  // Monitors:
  // exposure, gain, desired bin, current bin, FR3 AEGC enable, FR3 AEGC skip,
  // AEC and AGC (reg 175 bits 0 and 1), FPS, 
  void calcFrameRate();
  void getCurrentBins(); // desired and current
  void getCurrentExpGain();
  void getCurrentAEGC();

  void updateTrackbars(); // check for value change instead of using callback

 private:
  // The camera we're controlling
  FrEagleStereoSmartCam *ssc;

  int visible; // true if the window is visible
  int aegcMode; // AEGC, manual, etc. (if others)

  // variables to track framerate
  struct timeval currTv;
  double startTime;
  double lastTime;
  double thisTime;
  float framerate;
  float accumFPS;

  // variables to track monitor updates
  int mExp, sExp;
  int mGain, sGain;
  int mCBin, sCBin, mDBin, sDBin;

  // window name
  const char *windowName;
  // iplimage we draw info on. Could also be a FrImage to keep the interface clean.
  IplImage *monImg;
  CvFont monFont;
  CvScalar fontColor;

  int updateInterval; // number of frames between monitor/status updates
  int currFrameNum; // Frame count
  int lastUpdateNum; // frame number when monitor/status was last updated

  // track bar variables
  // could track last value for each of these and use it to update hardware when
  // the trackbar is changed without having to use a callback (which has no
  // generic pointer to data)
  int desiredBin, lastDesiredBin; // 1 to 64
  int manualExp, lastManualExp; // 1 to 480
  int manualGain, lastManualGain; // 16 to 64
  int matchQual, lastMatchQual; // 0 to 127

};

// callback functions aren't part of the class
// may not need any callbacks, actually. Maybe a mouse callback?

#endif
