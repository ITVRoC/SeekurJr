/****************************************************************************
 * Copyright (c) 2008-2009 by Focus Robotics.
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
#include "FrStereoSmartCamGUI.h"
//#include <sys/time.h>

/**
 * Constructor
 */
FrStereoSmartCamGUI::FrStereoSmartCamGUI(FrEagleStereoSmartCam *sscam, const char *name) {
  ssc = sscam;
  windowName = name;
  currFrameNum = 0;
  lastUpdateNum = 0;
  updateInterval = 10;
  framerate = 30.0;
  visible = 0;
  init();
  show();
}

/**
 * Destructor
 * Destroy window, free image, anything else used.
 */
FrStereoSmartCamGUI::~FrStereoSmartCamGUI() {

}

/**
 * Make the control window invisible. Keystroke commands still work as long as
 * the update function is called with new keystrokes.
 */
void FrStereoSmartCamGUI::hide(int type) {
  if(!visible) return;
  cvDestroyWindow(windowName);
  visible = 0;
}

/**
 * The control window is visible by default when this class is constructed, but
 * if the window is hidden with hide() above, call this function to show it 
 * again.
 */
void FrStereoSmartCamGUI::show(int type) {
  if(visible) return;
  cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
  cvMoveWindow(windowName, 0, 540);

  // add trackbar controls
  cvCreateTrackbar("Desired Bin", windowName, &desiredBin, 64, 0);
  cvCreateTrackbar("Manual Exp ", windowName, &manualExp, 480, 0);
  cvCreateTrackbar("Manual Gain", windowName, &manualGain, 64, 0);
  cvCreateTrackbar("Match Qual ", windowName, &matchQual, 128, 0);

  // draw the monitor window for the first time
  drawControlWindow();
  visible = 1;
}

/**
 * Allow the control window to be moved to an absolute postion on the screen to
 * enable programs to have particular window layouts.
 * If X and Y are saved then hiding and reshowing the window can put it back to
 * the same place it was. Not sure how to get coords that the user dragged the
 * window to, though.
 */
void FrStereoSmartCamGUI::move(int x, int y, int type) {
  cvMoveWindow(windowName, x, y);
}

/**
 * This function should be called once per frame or at a similar short regular
 * interval to update the gui. It serves as the heartbeat for this class.
 */
void FrStereoSmartCamGUI::update(int kp) {
  // only update every few frames!
  currFrameNum++;
  if((currFrameNum % updateInterval == 0) && visible) {
    lastUpdateNum = currFrameNum;
    // update monitors
    calcFrameRate();
    getCurrentBins();
    getCurrentExpGain();
    getCurrentAEGC();
    drawControlWindow();
    updateTrackbars();
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Protected Functions//////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize internal structures and variables when the class is first constructed.
 * This is only called by the constructor.
 * This code could be moved into the constructor it's self if it doesn't end up 
 * growing much larger.
 */
void FrStereoSmartCamGUI::init() {
  // Setup trackbar variables
  lastDesiredBin = desiredBin = 35;
  lastManualExp = manualExp = 0;
  lastManualGain = manualGain = 0;
  lastMatchQual = 55; // force updated with diff last val, or else show actual val!
  matchQual = 56;

  // create image and set up font for monitors
  monImg = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 3);
  float hscale = 0.5f;
  float vscale = 0.5f;
  float italicscale = 0.0f;
  int thickness = 1; 
  int line_type = 8;
  fontColor = CV_RGB(255, 0, 0);
  cvInitFont(&monFont, CV_FONT_VECTOR0, 
	     hscale, vscale, italicscale, thickness, line_type);

  // init fps calc
  gettimeofday(&currTv,0);
  thisTime = currTv.tv_sec + (currTv.tv_usec/1000000.0);
  lastTime = thisTime;
  startTime = thisTime;
}

/**
 * (Re)draw the text on the control window that shows current status.
 * It might be more efficient to just redraw items that change, but it doesn't
 * take a huge amount of CPU as it is so I haven't bothered to change it.
 */
void FrStereoSmartCamGUI::drawControlWindow() {
  // clear the image
  cvSetZero(monImg);
  int line=0;
  // write monitor values with puttext
  char *text = (char*)malloc(1024); // fixme: malloc only once and save as instance variable
  // write frame rate
  sprintf(text, "Frame Rate: %2.2f (%2.2f avg. since start)", framerate, accumFPS);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);

  // write AEGC settings
  sprintf(text, "AEGC Mode: %d", aegcMode);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);

  // write desired and current bins
  sprintf(text, "Desired Bin:   Right: %2d    Left: %2d", mDBin, sDBin);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  sprintf(text, "Current Bin:   Right: %2d    Left: %2d", mCBin, sCBin);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  //sprintf(text, "Current Bin Right: %2d", mCBin);
  //cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  //sprintf(text, "Desired Bin Right: %2d", mDBin);
  //cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);

  // write exposure and gain settings
  sprintf(text, "Exp:           Right: %3d   Left: %3d", mExp, sExp);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  sprintf(text, "Gain:          Right: %3d   Left: %3d", mGain, sGain);
  cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  //sprintf(text, "Right Exposure: %3d", mExp);
  //cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);
  //sprintf(text, "Right Gain:     %3d", mGain);
  //cvPutText(monImg, text, cvPoint(5,++line*20), &monFont, fontColor);

  free(text);
  // tell highgui to put the new image on the screen
  // this is only drawn when the higher level actually calls cvWait();
  cvShowImage(windowName, monImg);
}

/**
 *
 */
void FrStereoSmartCamGUI::calcFrameRate() {
  // should calculate over the last 10 frames and over all time
  // I think that calculating over 100 frames might be better, but I'm not sure
  // how to do that within this framework.
  gettimeofday(&currTv,0);
  lastTime = thisTime;
  thisTime = currTv.tv_sec + (currTv.tv_usec/1000000.0);
  framerate = updateInterval/(thisTime-lastTime);
  accumFPS = currFrameNum/(thisTime-startTime);
}

/**
 *
 */
void FrStereoSmartCamGUI::getCurrentBins() {
  // read values via ecam
  // read current and desired bins for both image sensors
  mCBin = ssc->getCurrentBin(0);
  sCBin = ssc->getCurrentBin(1);
  mDBin = ssc->getDesiredBin(0);
  sDBin = ssc->getDesiredBin(1);
}

/**
 *
 */
void FrStereoSmartCamGUI::getCurrentExpGain() {
  // read values via ecam
  // read current exposure and gain settings for both image sensors
  mExp = ssc->getCurrentExp(0);
  sExp = ssc->getCurrentExp(1);
  mGain = ssc->getCurrentGain(0);
  sGain = ssc->getCurrentGain(1);
}

/**
 *
 */
void FrStereoSmartCamGUI::getCurrentAEGC() {
  // read values via edev
  // AEGC enable, frame skip
  aegcMode = ssc->getAegcMode();
}

/**
 * Check to see if trackbars have changed and update params if they have.
 */
void FrStereoSmartCamGUI::updateTrackbars() {
  if(lastDesiredBin!=desiredBin) {
    lastDesiredBin = desiredBin;
    ssc->setCamBrightness(desiredBin);
  }

  if(lastManualExp!=manualExp) {
    lastManualExp = manualExp;
    ssc->setCamExposure(manualExp);
  }

  if(lastManualGain!=manualGain) {
    lastManualGain = manualGain;
    ssc->setCamGain(manualGain);
  }

  if(lastMatchQual!=matchQual) {
    lastMatchQual = matchQual;
    ssc->setMatchQualThresh(matchQual);
  }

}


