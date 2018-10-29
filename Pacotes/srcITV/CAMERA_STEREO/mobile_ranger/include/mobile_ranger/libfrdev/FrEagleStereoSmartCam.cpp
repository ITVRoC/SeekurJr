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
 * Creation_Date:  Sun Apr  1 2007
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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include "FrEagleStereoSmartCam.h"

/**
 * Primary Constructor.
 * Optional arguments should support device selection when there are multiple 
 * Eagle devices in the system. It should also support calib file specification.
 *
 * Alternate constructor (FrEagleStereoSmartCam(FrOptArgs *args); // prob the normal one!
 * Alternate constructor (FrEagleStereoSmartCam(FrEagleDevice *edev, FrStereoRectMap *rmap);
 */
FrEagleStereoSmartCam::FrEagleStereoSmartCam(int id) {
  // first clear all pointers to zero!
  initHWFrameList();
  vtxTransferEnable = 0;
  rectProcEnable = 1;
  corrProcEnable = 1;
  curFrameNum = 0;
  reqBin = 35;
  args=0;
  // Set default image types per DMA channel
  for(int i=0; i<4; i++) { chanType[i]=ITYPE_NONE; chanImg[i]=0; }

  // Create the EagleDevice
  edev = new FrEagleDevice(id);
  // Create the EagleStereoCam
  ecam = new FrEagleStereoCam(edev);
  ecam->resetCameras();
  // Create the StereoRectMap and load the remap data into Eagle
  rmap = new FrStereoRectMap(findCalibFile());
  edev->writeMEM(EAGLE_RECT_MAP_BASE, rmap->getFr3RemapStructLen(), rmap->getFr3RemapStruct());
  // Run the processing pipe once to prime the pump
  // might be nice to clear all frames to black before doing this
  initProcPipe();
  //start the video stream
  ecam->initCamera();
  setAegcMode(3);
  setAegcSkip(3);
}

FrEagleStereoSmartCam::FrEagleStereoSmartCam(FrOptArgs *a) {
  // first clear all pointers to zero!
  initHWFrameList();
  vtxTransferEnable = 0;
  rectProcEnable = 1;
  corrProcEnable = 1;
  curFrameNum = 0;
  reqBin = 35;
  // save args in a instance variable so findCalibFile() can make use of it!
  args = a;

  // Set default image types per DMA channel
  for(int i=0; i<4; i++) { chanType[i]=ITYPE_NONE; chanImg[i]=0; }

  // Create the EagleDevice
  edev = new FrEagleDevice(*args);
  // Create the EagleStereoCam
  ecam = new FrEagleStereoCam(edev);
  ecam->resetCameras();
  // Create the StereoRectMap and load the remap data into Eagle
  rmap = new FrStereoRectMap(findCalibFile());
  edev->writeMEM(EAGLE_RECT_MAP_BASE, rmap->getFr3RemapStructLen(), rmap->getFr3RemapStruct());
  // Run the processing pipe once to prime the pump
  // might be nice to clear all frames to black before doing this
  initProcPipe();
  //start the video stream
  ecam->initCamera();
  setAegcMode(3);
  setAegcSkip(3);
}

/**
 * Destructor
 */
FrEagleStereoSmartCam::~FrEagleStereoSmartCam() {
  // Shutdown and delete the rectMap, EagleStereoCam, EagleDevice, and all 
  // images allocated during operation.
  // reset and power down the camera as much as possible
  //ecam->resetCameras(); // done by ecam
  delete(rmap);
  delete(ecam);
  delete(edev);
  // FIXME: delete any FrImages created
  // I create images with custom malloc'd buffers in setChanType--DEFINITE MEMORY LEAK!
  for(int i=0; i<4; i++) { 
    if(chanImg[i]!=0) {
      free(chanImg[i]->getDptr()); // must separately free the data since we malloc'd it separately
      delete(chanImg[i]);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Camera Properties
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**
 * Camera image properties control
 */
void FrEagleStereoSmartCam::setCamBrightness(int b) {
  // call to ecam to set desired bin, AEGC turned on
  // desired bin must be 1-64
  // FIXME: save result
  ecam->setAEGCDesiredBin(b);
  reqBin = b;
}

void FrEagleStereoSmartCam::setDesiredBin(int b, int sel) {
  if(sel==0) {

  } else if(sel==1) {

  } else if(sel==-1) {

  } else {

  }
}

// this should normally return the last value set, maybe with a debug option to force reread
// FIXME: add a debug option to force a reread
int FrEagleStereoSmartCam::getDesiredBin(int sel) {
  //return ecam->getCamCSR(MT9V022_AEC_AGC_DESIRED_BIN, sel);
  return reqBin;
}

// this could cache the result and just return the cached value if called again within n frames
// This is a calculated measure of image brightness, generated by the image sensor
// We could calculate other measures of brightness in the fpga or in software
int FrEagleStereoSmartCam::getCurrentBin(int sel) {
  if(sel==0) {
    curBinR = ecam->getCamCSR(MT9V022_AGC_AEC_CURRENT_BIN, sel);
    return curBinR;
  } else {
    curBinL = ecam->getCamCSR(MT9V022_AGC_AEC_CURRENT_BIN, sel);
    return curBinL;
  }
}

/**
 * Exposure Control
 * Camera image properties control. Just passes through to EagleStereoCam object
 * Exposure values can be between 1 and 480 lines
 */
void FrEagleStereoSmartCam::setCamExposure(int e) { setExp(e, -1); } // depreciated
int FrEagleStereoSmartCam::getCurrentExp(int sel) { return getExp(sel); } // depreciated

void FrEagleStereoSmartCam::setExp(int e, int sel) {
  ecam->setExp(e, sel);
  if(sel==-1) { curExp = curExpR = curExpL = e; }
  else if(sel==0) { curExpR = e; }
  else if(sel==1) { curExpL = e; }
}

int FrEagleStereoSmartCam::getExp(int sel) {
  return ecam->getExp(sel); // should this return the last programmed value, or reread the sensor???
}

/**
 * Analog Gain Control
 * Camera image properties control. Just passes through to EagleStereoCam object
 */
void FrEagleStereoSmartCam::setCamGain(int g) { setAGain(g); } // depreciated
int FrEagleStereoSmartCam::getCurrentGain(int sel) { return getAGain(sel); } // depreciated

void FrEagleStereoSmartCam::setAGain(int g, int sel) {
  ecam->setAGain(g, sel);
  if(sel==-1) { curGain = curGainR = curGainL = g; }
  else if(sel==0) { curGainR = g; }
  else if(sel==1) { curGainL = g; }
}

int FrEagleStereoSmartCam::getAGain(int sel) {
  return ecam->getAGain(sel); // should this return the last programmed value, or reread the sensor???
}

/**
 *
 */
void FrEagleStereoSmartCam::setAegcMode(int mode) {
  ecam->setAegcMode(mode);
  aegcMode = mode;
  // FIXME: refresh exp, gain, bin when switching to mode 3--software algorithims need up to date values
}

void FrEagleStereoSmartCam::setI2cReg(int offset, int dev, int value) {ecam->setCamCSR(offset, value, dev);} // dev -1 bc
int FrEagleStereoSmartCam::getI2cReg(int offset, int dev) {return ecam->getCamCSR(offset, dev);}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Rectification Processor Properties
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**
 * This allows the rectification map that was loaded when the object was
 * constructed to be changed to something else. This can be useful when 
 * calibrating the camera so that the newly generated rect map can be loaded
 * without shutting down and reinitializing the camera. It can also be used for
 * automatic live recalibration schemes that periodically upate the rect map
 * while running.
 */
void FrEagleStereoSmartCam::setRectMap(FrStereoRectMap *r) {
  // The data from the new rect map should be copied into the old one, if
  // possible. 
  // FIXME: operator= isn't currently assigned in the rectmap class; do that!
  // Should the old map be destroyed and a new map constructed with the copy constructor?
  *rmap = *r; // copy all params and maps
  edev->writeMEM(EAGLE_RECT_MAP_BASE, rmap->getFr3RemapStructLen(), rmap->getFr3RemapStruct());
}
FrStereoRectMap *FrEagleStereoSmartCam::getRectMap() {
  return rmap;
}
// FIXME: calling this function should mark the rect map bad somehow!
void FrEagleStereoSmartCam::setRectData(int* data, int len) {
  edev->writeMEM(EAGLE_RECT_MAP_BASE, len, data);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Correlation Processor Properties
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**
 * Set match quality threshold CSR from float input
 * Exponent or shift field is 4 bit two's compliment -7 to 7
 * Significand is 0-15 but msb must always be 1 (as in 1.xxx)
 */
void FrEagleStereoSmartCam::setMatchQualThresh(float t) {
  int mqc_regval = 0; // // 3:0 shift, 7:4 man, 8 en
  if(t<0.0 || t>128.0) return;
  if(t>0.0001) {
    int exponent = ilogbf(t);
    // significant() returns a number in the range 1,2. I mult by 8 to make 8-15
    // theoretically, I could pass 0-15, but the msb is actually redundant
    int signif = (int)(significandf(t) * 8.0);
    mqc_regval = (0x100 | ((signif & 0x0f)<<4) | (exponent & 0x0f));
  }
  //printf("setting mqc float\n");
  edev->setCSR(EAGLE_CSR_STEREO_MQC_CTL, mqc_regval);
}

/**
 * Set match quality threshold CSR from int input
 * FIXME: int and float match qual thresh methods should be made to work together
 * better. Right now the getMatchQualThresh only returns values set by the int
 * version. Maybe there should be a getMatchQualThreshFloat and ...Int version
 * and they should both generate their return values based on what is actually 
 * programmed in the hardware.
 */
void FrEagleStereoSmartCam::setMatchQualThresh(int t) {
  matchQualThresh = t;
  int mqc_regval = 0; // CSR format: 3:0 shift, 7:4 man, 8 en
  if(t<0 || t>127) return;
  if(t>0) {
    int exponent = ((t&0x78)>>3) - 7;
    //             en       significand              exponent
    mqc_regval = (0x100 | 0x080 | ((t&0x07)<<4) | (exponent & 0x0f));
  }
  //printf("setting mqc int %x\n", mqc_regval);
  edev->setCSR(EAGLE_CSR_STEREO_MQC_CTL, mqc_regval);
}

void FrEagleStereoSmartCam::setDispSearchRange(int r) {}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Streaming Data Controller
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**
 * Write an image of a particular type into the hardware. This can be used when
 * the calib and stereo cores are needed to process external images not created
 * by the attached stereo camera. Raw or rectified images can be written into
 * the hardware and disparity can be retrieved.
 */
void FrEagleStereoSmartCam::setImg(FrImageType type, FrImage img) {
  
}


/**
 * Read an image of a particular type from the hardware. This will be done using
 * the relatively slow memory-mapped read rather than the DMA you get from grab
 * and retrieve.
 */
FrImage *FrEagleStereoSmartCam::getImg(FrImageType type) {
  return 0; 
}


/**
 *
 */
void FrEagleStereoSmartCam::setChanType(int c, FrImageType t) {
  if(c<0 || c>3) {
    printf("FrEagleStereoSmartCam::setChanType: channel number out of range\n");
    return;
  }
  chanType[c] = t;
  // (re)alloc image if necessary
  if(chanImg[c]==0) {
    // stride of image must be 768 to work with EagleDevice retreiveFrame() so I must build the FrImage manually
    uchar *buf = (uchar*)malloc(368640);
    chanImg[c] = new FrImage(buf, 768, 752, 480, 1, 0);
  }
}


/**
 * Initiate a DMA transfer for a particular image from the processor hardware.
 */
void FrEagleStereoSmartCam::grabImg(FrImageType type) {
  // Wait for all previous running hw to complete (?)
  // Kick off DMA transfer for requested type
  // Use the pre-allocated channel for that type if possible, but if a channel
  // hasn't been allocated for that type then allocated one.
  // If the type ITYPE_ALL is passed, grab for all 4 channels--or at least all
  // with valid types associated. Any with ITYPE_NONE won't be grabbed.

  // need a function to reverse map: return a channel when passed a type: getChanForType(type)
  // need a function to allocate a new channel for a type if nothing matches
  // Ideally, the new channel will be one not currently active, so I need to track active channels
  // Initially, I can just refuse to grab if a channel hasn't been allocated for that type
  // That will make it quite a bit simpler to get started.
  int chan = getChanForType(type);
  if(chan==-1) return;
  //int src = 0; // FIXME: how do I set src correctly for a given type!
  // I need another function to select the appropriate pipe register depending on type.
  // I might need to do some work with how pipe registers are set up in order to do that.
  edev->grabFrame(chan, getCompleteHWFrame(type));
}

void FrEagleStereoSmartCam::grabImg(int chan) {
  if(chan>3 || chan<-1) {
    // FIXME: Error! Illegal channel selected
  } else if (chan==-1) {
    // loop, grabbing all channels not set to ITYPE_NONE. Call grabImg for each.
    for(int i=0; i<4; i++) {
      if(chanType[i]!=ITYPE_NONE) { 
	// FIXME: set bit on grab so that only the first retrieve actually has to copy
	// additional retrieves (if done) just return pointer again
	edev->grabFrame(i, getCompleteHWFrame(chanType[i])); 
      }
    }
  } else {
    // FIXME: set bit on grab so that only the first retrieve actually has to copy
    edev->grabFrame(chan, getCompleteHWFrame(chanType[chan]));
  }
}

/**
 * Retrieve an image from a previously initiated DMA. 
 */
FrImage *FrEagleStereoSmartCam::retrieveImg(FrImageType type) {
  // Wait for DMA of given type to complete--EagleDevice does this!
  // Select image to read the data into based on the type--or should I just have 1 image per channel?
  // Update image data: just call edev->retrieveFrame() with the channel from type and idata for correct image based on type
  // Return appropriate image for specified type
  return 0; 
}
FrImage *FrEagleStereoSmartCam::retrieveImg(int chan) {
  if(edev->retrieveFrame(chan, chanImg[chan]->getDptr())) {
    return 0; // returns null img ptr on error
  } else {
    return chanImg[chan];
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Overall Properties and Methods
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void FrEagleStereoSmartCam::setMode(int streaming, int lowlatency) {
  // Maybe there should be an enum here as listed below, instead of two ints
}


/**
 * Perform processing according to mode settings
 * The call to run() might be performed implicitly by grabImg() in some modes.
 * Alternately, the call to run() could block until the run is complete, only a non-blocking run could be called by grabImg()
 * Mode 0: FR_SSC_PIPELINED_STREAMING
 * Mode 1: FR_SSC_PIPELINED_ONESHOT
 * Mode 2: FR_SSC_LOWLATENCY_STREAMING
 * Mode 3: FR_SSC_LOWLATENCY_ONESHOT
 * Mode 4: FR_SSC_RECTIFY_ONESHOT
 * Mode 5: FR_SSC_CORRELATE_ONESHOT
 * Mode 6: FR_SSC_RECTCORR_ONESHOT
 */
int FrEagleStereoSmartCam::run() {
  curFrameNum++;
  stepProcPipe();
  if(aegcMode==3) { runSoftAEGC(); }
  if(aegcMode==4) { runIndSoftAegc(); }
  return 0; 
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
/******************************************************************************/
/* Internal Protected Functions */
/******************************************************************************/
/******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Return new Exposure from desired brightness change and current Exposure
int calcNewExposure(float dBGain, int curExp, int maxExp) {
  int calcExp = (int)(curExp*exp10(dBGain/10.0));
  if(calcExp>maxExp) calcExp = maxExp;
  if(calcExp<1) calcExp = 1;
  return(calcExp);
}

// Return new Analog Gain from desired brightness change and current Analog Gain
int calcNewAGain(float dBGain, int curAGain) {
  int calcAGain = (int)(curAGain + (dBGain * 4.0));
  if(calcAGain>64) calcAGain = 64;
  if(calcAGain<16) calcAGain = 16;
  return(calcAGain);
}

// calc new exp and gain for either left or right side
void calcNewAegcParams(int reqBin, int curBin, int curExp, int curGain, int *newExp, int *newGain) {
  int aegcVerbose = 0;
  // determine difference
  int diff = reqBin - curBin;
  // calculate required gain or attenuation to make the image close to the correct brightness
  float dBGain = diff/8.0;
  if(curBin<4) dBGain = dBGain*2.0;
  if(curBin<8) dBGain = dBGain*2.0;
  if(curBin>40) dBGain = dBGain*0.75;

  // check for small adjustments  
  // maybe also check for pegged controls so there's no point in doing a full computation
  if((diff < 5 && diff > -8) && (curExp+diff>60 && curExp+diff<480)) {
    // minor adjustment, just add diff to exposure unless that would cause an overflow
    // maybe check for overflow in the if clause
    //newGain = curGain;
    *newExp = curExp + diff;
    if(aegcVerbose) printf("AEGC: Minor exp adjustment: %d\n", diff);
  } else {
    if(aegcVerbose) printf("AEGC: gain=%f diff=%d ", dBGain, diff);
    if(curExp>=100 && curExp<=400) { // Exp in desired range
    // If Exp in desired range and AGain not pegged, adj AGain
    // If Exp in desired range and AGain pegged, adj Exp
      if((diff>0 && curGain<64) || (diff<0 && curGain>16)) { // AGain not pegged
	// adjust AGain
	*newGain = calcNewAGain(dBGain, curGain);
	if(aegcVerbose) printf("Desired exp range, Adj Gain to %d\n", *newGain);
      } else { // AGain pegged
	// adjust Exposure--max 480
	*newExp = calcNewExposure(dBGain, curExp, 480);
	if(aegcVerbose) printf("Desired exp range, Adj exp to %d\n", *newExp);
      }
    } else { // Exp not in desired range
    // If Exp not in desired range and this brings it closer, adjust Exp (with max)
    // If Exp not in desired range and this brings it farther, and AGain not pegged, adjust AGain
    // If Exp not in desired range and this brings it farther, but AGain pegged, adj Exp
      if((diff>0 && curExp<100) || (diff<0 && curExp>400)) { // brings Exp closer to desired range
	// adjust Exposure--max 360
	*newExp = calcNewExposure(dBGain, curExp, 360);
	if(aegcVerbose) printf("Out of exp range, Adj exp closer to %d\n", *newExp);
      } else { // brings Exp further away from desired range
	if((diff>0 && curGain<64) || (diff<0 && curGain>16)) { // AGain not pegged
	  // adjust AGain
	  *newGain = calcNewAGain(dBGain, curGain);
	  if(aegcVerbose) printf("Out of exp range, Adj gain to %d\n", *newGain);
	} else { // AGain pegged
	  // adjust Exposure--max 480
	  *newExp = calcNewExposure(dBGain, curExp, 480);
	  if(aegcVerbose) printf("Out of exp range, Adj exp further to %d\n", *newExp);
	}
      }
    }
  }

}


/**
 * RunIndSoftAegc
 * This function calculates the new exposure and gain settings based on the
 * current settings and the current and desired brightness of the images.
 *
 * FIXME:
 * Currently, this is just a copy of the old function. Update to adjust each image sensor independently
 * 
 */
void FrEagleStereoSmartCam::runIndSoftAegc() {
  // handle frame skip: just return if it isn't time to run again
  if(curFrameNum%aegcSkip) return;

  // make sure the camera params are current
  int cbin0 = getCurrentBin(0);
  int cbin1 = getCurrentBin(1);
  int dbin = getDesiredBin(0);

  // calculate new parameters
  calcNewAegcParams(dbin, cbin0, curExpR, curGainR, &newExpR, &newAGainR);
  calcNewAegcParams(dbin, cbin1, curExpL, curGainL, &newExpL, &newAGainL);

  // program the new exposure and gain
  setExp(newExpR, 0);
  setExp(newExpL, 1);
  setAGain(newAGainR, 0);
  setAGain(newAGainL, 1);
}

/**
 * RunSoftAEGC
 * This function does the whole thing: make sure current bins are up to date,
 * calc new parameters, program them back. To be called from run() when in the
 * appropriate mode.
 */
void FrEagleStereoSmartCam::runSoftAEGC() {
  // handle frame skip: just return if it isn't time to run again
  if(curFrameNum%aegcSkip) return;
  //if(curFrameNum%3) return;

  // make sure the camera params are current
  getCurrentBin(0);
  getCurrentBin(1);
  //printf("Current Bins right=%d, left=%d, requested=%d\n", curBinR, curBinL, reqBin);
  // calculate the new camera exposure and gain from current params
  calcNextParmsAEGC2();
  //printf("Call to calcNextParamsAegc results in newExp=%d and newGain=%d\n", newExp, newGain);

  // program the new exposure and gain
  setCamExposure(newExp);
  setCamGain(newGain);
}

/**
 * CalcNextParmsAEGC
 * Assume currBinR and currBinL are current, reqBin is what is desired
 * curExp and curGain are what was last programmed, generate newExp and newGain
 * Implement Jason's algorithm first and then refine
 * Possible Refinements:
 * consider both left and right current bin so neither will max if lens cap is on
 * better oscillation avoidance
 * quicker adjustment in the case of huge bin differences
 */
void FrEagleStereoSmartCam::calcNextParmsAEGC() {
  // determine difference
  int diff = reqBin - curBinR;

  // set new gain -- between 16 and 64
  if(curExp==480) {
    // only increase gain if exposure is already at 480
    // if exposure is at 480 and too bright, reduce gain to 16 first
    newGain = curGain + diff;
  }
  if(newGain>64) newGain = 64;
  if(newGain<16) newGain = 16;

  // set new exposure -- between 1 and 480
  if(newGain > 16) {
    // don't reduce exposure from 480 if gain is still > 16
  } else if(curExp < 16) {
    // only set new exposure if certain thresholds are met if current exposure is between 1 and 15
    //float binThresh = (curBinR * ((1/curExp) + 1));
    if(reqBin > curBinR) {
      newExp = curExp + 1;
    } else if(reqBin < curBinR) {
      newExp = curExp - 1;
    }
  } else {
    // otherwise, just add diff to old exposure to get new exposure, clamping at 480
    newExp = curExp + diff;
  }
  if(newExp>480) newExp = 480;
  if(newExp<1) newExp = 1;

  // Output of this function is newExp and newGain
}

/**
 *
 */
void FrEagleStereoSmartCam::calcNextParmsAEGC2() {
  // determine difference
  int diff = reqBin - curBinR;
  // calculate required gain or attenuation to make the image close to the correct brightness
  float dBGain = diff/8.0;
  if(curBinR<4) dBGain = dBGain*2.0;
  if(curBinR<8) dBGain = dBGain*2.0;
  if(curBinR>40) dBGain = dBGain*0.75;

  // check for small adjustments  
  // maybe also check for pegged controls so there's no point in doing a full computation
  if((diff < 5 && diff > -8) && (curExp+diff>60 && curExp+diff<480)) {
    // minor adjustment, just add diff to exposure unless that would cause an overflow
    // maybe check for overflow in the if clause
    //newGain = curGain;
    newExp = curExp + diff;
    if(aegcVerbose) printf("AEGC: Minor exp adjustment: %d\n", diff);
  } else {
    if(aegcVerbose) printf("AEGC: gain=%f diff=%d ", dBGain, diff);
    if(curExp>=100 && curExp<=400) { // Exp in desired range
    // If Exp in desired range and AGain not pegged, adj AGain
    // If Exp in desired range and AGain pegged, adj Exp
      if((diff>0 && curGain<64) || (diff<0 && curGain>16)) { // AGain not pegged
	// adjust AGain
	newGain = calcNewAGain(dBGain, curGain);
	if(aegcVerbose) printf("Desired exp range, Adj Gain to %d\n", newGain);
      } else { // AGain pegged
	// adjust Exposure--max 480
	newExp = calcNewExposure(dBGain, curExp, 480);
	if(aegcVerbose) printf("Desired exp range, Adj exp to %d\n", newExp);
      }
    } else { // Exp not in desired range
    // If Exp not in desired range and this brings it closer, adjust Exp (with max)
    // If Exp not in desired range and this brings it farther, and AGain not pegged, adjust AGain
    // If Exp not in desired range and this brings it farther, but AGain pegged, adj Exp
      if((diff>0 && curExp<100) || (diff<0 && curExp>400)) { // brings Exp closer to desired range
	// adjust Exposure--max 360
	newExp = calcNewExposure(dBGain, curExp, 360);
	if(aegcVerbose) printf("Out of exp range, Adj exp closer to %d\n", newExp);
      } else { // brings Exp further away from desired range
	if((diff>0 && curGain<64) || (diff<0 && curGain>16)) { // AGain not pegged
	  // adjust AGain
	  newGain = calcNewAGain(dBGain, curGain);
	  if(aegcVerbose) printf("Out of exp range, Adj gain to %d\n", newGain);
	} else { // AGain pegged
	  // adjust Exposure--max 480
	  newExp = calcNewExposure(dBGain, curExp, 480);
	  if(aegcVerbose) printf("Out of exp range, Adj exp further to %d\n", newExp);
	}
      }
    }
  }
}

/**
 * Initialize all hardware frame slots to their designated first buffer
 * FIXME: properly document the way frame slots are used in hw for pipelined mode
 */
int FrEagleStereoSmartCam::initHWFrameList() {
  completeHWFrames[0] = activeHWFrames[0] = 0;
  completeHWFrames[1] = activeHWFrames[1] = 4;
  completeHWFrames[2] = activeHWFrames[2] = 8;
  completeHWFrames[3] = activeHWFrames[3] = 11;
  completeHWFrames[4] = activeHWFrames[4] = 14;
  return 0;
}

/**
 * FIXME: though this produces a smooth video output, it retreives right and
 * disp images that are one frame out of sync. I actually need a pipe that is
 * 3 deep to account for the 3 stages in the hw processing pipe.
 */
int FrEagleStereoSmartCam::stepHWFrameList() {
  // Find current camera frame
  int vrxCurrFrame = edev->getCSR(EAGLE_CSR_VRX_FRAME_CNT);
  if(vrxCurrFrame==0) {vrxCurrFrame = 3;} else {vrxCurrFrame--;}
  if(vrxCurrFrame==activeHWFrames[0]) return 1; // current frame not finished

  // copy current active frames to complete
  for(int i=0; i<5; i++) { completeHWFrames[i] = activeHWFrames[i]; }
    // set up rect input as most recently completed camera frames
  activeHWFrames[0] = vrxCurrFrame + 0;
  activeHWFrames[1] = vrxCurrFrame + 4;
  // step rect out/corr in frames through 3 deep circular buffers
  if(++activeHWFrames[2]==11) { activeHWFrames[2] = 8; }
  if(++activeHWFrames[3]==14) { activeHWFrames[3] = 11; }
  // step disp out through 2 deep circular buffer
  if(++activeHWFrames[4]==16) { activeHWFrames[4] = 14; }
  return 0;
}

/**
 *
 */
int FrEagleStereoSmartCam::getActiveHWFrame(FrImageType t) {
  switch(t) {
  case ITYPE_RIGHT_RAW: return activeHWFrames[0];
  case ITYPE_LEFT_RAW: return activeHWFrames[1];
  case ITYPE_RIGHT_RECT: return activeHWFrames[2];
  case ITYPE_LEFT_RECT: return activeHWFrames[3];
  case ITYPE_FINAL_DISP: return activeHWFrames[4];
  default: return -1;
  }
}

/**
 *
 */
int FrEagleStereoSmartCam::getCompleteHWFrame(FrImageType t) {
  switch(t) {
  case ITYPE_RIGHT_RAW: return completeHWFrames[0];
  case ITYPE_LEFT_RAW: return completeHWFrames[1];
  case ITYPE_RIGHT_RECT: return completeHWFrames[2];
  case ITYPE_LEFT_RECT: return completeHWFrames[3];
  case ITYPE_FINAL_DISP: return completeHWFrames[4];
  default: return -1;
  }
}

/**
 *
 */
void FrEagleStereoSmartCam::initProcPipe() {
  // Initiate rectification processing
  edev->setCSR(EAGLE_CSR_CALIB_RIGHT_SRC, getActiveHWFrame(ITYPE_RIGHT_RAW));
  edev->setCSR(EAGLE_CSR_CALIB_LEFT_SRC, getActiveHWFrame(ITYPE_LEFT_RAW));
  edev->setCSR(EAGLE_CSR_CALIB_RIGHT_DST, getActiveHWFrame(ITYPE_RIGHT_RECT));
  edev->setCSR(EAGLE_CSR_CALIB_LEFT_DST, getActiveHWFrame(ITYPE_LEFT_RECT));
  edev->setCSR(EAGLE_CSR_CALIB_ENABLE, 3);

  // Initiate correlation processing
  edev->setCSR(EAGLE_CSR_STEREO_RIGHT_SRC, getCompleteHWFrame(ITYPE_RIGHT_RECT)); 
  edev->setCSR(EAGLE_CSR_STEREO_LEFT_SRC, getCompleteHWFrame(ITYPE_LEFT_RECT)); 
  edev->setCSR(EAGLE_CSR_STEREO_DISP_DST, getActiveHWFrame(ITYPE_FINAL_DISP));
  edev->setCSR(EAGLE_CSR_STEREO_ENABLE, 3);

  // Enable output transfers to the ref design if we're in devsys mode
  if(vtxTransferEnable) {
    int vtxDispSrc = getCompleteHWFrame(ITYPE_FINAL_DISP);
    int vtxRightSrc = getCompleteHWFrame(ITYPE_RIGHT_RECT);
    int vtxCmdWord = 0x4040 | (vtxDispSrc<<8) | vtxRightSrc;
    edev->setCSR(EAGLE_CSR_VTX_CTL, vtxCmdWord);
  }

}

/**
 *
 *
 */
void FrEagleStereoSmartCam::stepProcPipe() {
  // Wait for all previous running hw to complete
  // This includes procs and image transfers. I could potentially just wait for procs to be done,
  // but would run the (slight) risk of the next proc run corrupting the still transferring frame.
  // This is probably very unlikely, but I'm running in very conservative mode for now.
  // FIXME: or at least figure me out! Why does waitProcsDone cause jumping rect frames
  // while waitHWIdle() keeps the video smooth? I think that the only difference is
  // that waitHWIdle() waits for DMA to be finished, but the retreive calls I just made
  // should have done that!!!
  //waitProcsDone();
  waitHWIdle();

  int timeout = 0;
  while(stepHWFrameList()) {
    usleep(1000);
    timeout++;
    if(timeout>250) {
      printf("FrEagleStereoSmartCam: Timeout waiting for camera frame to increment!\n");
      ecam->resetCameras();
      usleep(250000);
      ecam->initCamera();
    }
  }

  // Initiate rectification processing
  if(rectProcEnable) {
    edev->setCSR(EAGLE_CSR_CALIB_RIGHT_SRC, getActiveHWFrame(ITYPE_RIGHT_RAW));
    edev->setCSR(EAGLE_CSR_CALIB_LEFT_SRC, getActiveHWFrame(ITYPE_LEFT_RAW));
    edev->setCSR(EAGLE_CSR_CALIB_RIGHT_DST, getActiveHWFrame(ITYPE_RIGHT_RECT));
    edev->setCSR(EAGLE_CSR_CALIB_LEFT_DST, getActiveHWFrame(ITYPE_LEFT_RECT));
    edev->setCSR(EAGLE_CSR_CALIB_ENABLE, 3);
  }
  // Initiate correlation processing
  if(corrProcEnable) {
    edev->setCSR(EAGLE_CSR_STEREO_RIGHT_SRC, getCompleteHWFrame(ITYPE_RIGHT_RECT)); 
    edev->setCSR(EAGLE_CSR_STEREO_LEFT_SRC, getCompleteHWFrame(ITYPE_LEFT_RECT)); 
    edev->setCSR(EAGLE_CSR_STEREO_DISP_DST, getActiveHWFrame(ITYPE_FINAL_DISP));
    edev->setCSR(EAGLE_CSR_STEREO_ENABLE, 3);
  }
  // Enable output transfers to the ref design if we're in devsys mode
  if(vtxTransferEnable) {
    int vtxDispSrc = getCompleteHWFrame(ITYPE_FINAL_DISP);
    int vtxRightSrc = getCompleteHWFrame(ITYPE_RIGHT_RECT);
    int vtxCmdWord = 0x4040 | (vtxDispSrc<<8) | vtxRightSrc;
    edev->setCSR(EAGLE_CSR_VTX_CTL, vtxCmdWord);
  }
}

/**
 * This function polls Eagle hardware until all functions are idle.
 * It should be called in single shot mode to synchronize with the end of each
 * frame.
 */
void FrEagleStereoSmartCam::waitHWIdle() {
  if(rectProcEnable && edev->waitCSR(EAGLE_CSR_CALIB_STATUS, 0x01, 0x01, 250)) {
    printf("Timeout while waiting for calib to complete!\n");
  }

  if(corrProcEnable && edev->waitCSR(EAGLE_CSR_STEREO_STATUS, 0x01, 0x01, 250)) {
    printf("Timeout while waiting for stereo to complete!\n");
  }

  // Wait for VTX transfers to complete if we're in devsys mode!
  if(vtxTransferEnable && edev->waitCSR(EAGLE_CSR_VTX_CTL, 0x8080, 0x8080, 250)) {
    printf("Timeout while waiting for vtx to complete! %x\n", edev->getCSR(EAGLE_CSR_VTX_CTL));
  }

  if(edev->waitCSR(EAGLE_CSR_DMAC_STATUS, 0x000, 0xf00, 250)) {
    printf("Timeout while waiting for PCI DMA to complete!\n");
  }

  // Clear all enables now that hw is idle
  edev->setCSR(EAGLE_CSR_CALIB_ENABLE, 0);
  edev->setCSR(EAGLE_CSR_STEREO_ENABLE, 0);
  edev->setCSR(EAGLE_CSR_CALIB_STATUS, 0);
  edev->setCSR(EAGLE_CSR_STEREO_STATUS, 0);
  edev->setCSR(EAGLE_CSR_VTX_CTL, 0x0400);

}


void FrEagleStereoSmartCam::startRectProc() {

}

void FrEagleStereoSmartCam::waitRectDone() {
  if(edev->waitCSR(EAGLE_CSR_CALIB_STATUS, 0x01, 0x01, 250)) {
    printf("Timeout while waiting for calib to complete!\n");
  }
  edev->setCSR(EAGLE_CSR_CALIB_ENABLE, 0);
}

void FrEagleStereoSmartCam::startCorrProc() {

}

void FrEagleStereoSmartCam::waitCorrDone() {
  if(edev->waitCSR(EAGLE_CSR_STEREO_STATUS, 0x01, 0x01, 250)) {
    printf("Timeout while waiting for stereo to complete!\n");
  }
  edev->setCSR(EAGLE_CSR_STEREO_ENABLE, 0);
}

void FrEagleStereoSmartCam::waitProcsDone() {
  waitRectDone();
  waitCorrDone();
}

/**
 * Search the filesystem for the *.calib file that goes with the camera this 
 * class is currently connected to. Search in a particular order depending on
 * command line options and environment variables.
 * I'm not sure if this class is the right location for this function. Perhaps
 * it would be more appropriate to put it in the FrStereoRectMap or elsewhere.
 *
 * This class depends on ecam->getCamSerNum() and on an FrOptArgs. If it is moved
 * into the FrStereoRectMap class, these two items will need to be passed in for
 * it to work properly. Maybe I should start passing them in here instead of 
 * depending on being in this class so it's requirements are very clear.
 *
 * To be even more generic, it could pass in two strings: the calib-file argument
 * (or null) from the command line and the sernum string (or null) from the
 * camera serial number. Of course I don't want passing things that way to turn
 * into a whole bunch of extra code so maybe passing the sernum int* and the 
 * FrOptArgs* will be the right way to go. These could be passed into the stero
 * rect map constructor or to an explicit call to findCalibFile in that class.
 */
const char* FrEagleStereoSmartCam::findCalibFile() {
  // Get the camera serial number and then look in various places in decreasing
  // order of priority.
  int *sernum = ecam->getCamSerNum();
  char cfname[32];
  char *cfpath;
  struct stat finfo;
  // NOTE: only using master image sensor serial number to determine calib filename
  sprintf(cfname, "%8.8x%8.8x.calib", sernum[1], sernum[0]);
  //printf("Calib file name is %s\n", cfname);

  // FIRST, check for optArgs
  // If there are optArgs, use the calib file given, or look in the calib dir given
  if(args!=0){
    printf("FrEagleStereoSmartCam::findCalibFile: valid args set; calib-file check is %d\n", args->checkArg("calib-file"));
    if(args->checkArg("calib-file") == 2) {
      printf("FrEagleStereoSmartCam::findCalibFile: Using calibration file %s\n", args->getCharArg("calib-file"));
      return args->getCharArg("calib-file");
    }
  }

  // SECOND, if no calib file found, look in the dir pointed to by FOCUS_ROBOTICS_CALIB_DIR
  //  Maybe also look for a file matching FOCUS_ROBOTICS_CALIB_FILE, but the dir
  //  would be used most often.
  if((cfpath = getenv("FOCUS_ROBOTICS_CALIB_FILE"))) {
    sprintf(calibFileName, "%s", cfpath);
    if(stat(calibFileName, &finfo)==0) {
      // FIXME: should verify that it's a regular file and readable
      printf("FrEagleStereoSmartCam::findCalibFile: Using calibration file %s\n", calibFileName);
      return calibFileName;
    }
    printf("FrEagleStereoSmartCam::findCalibFile: WARNING! The environment variable FOCUS_ROBOTICS_CALIB_FILE\n");
    printf("is set to %s, but the file can't be accessed for calib data.\n", calibFileName);
  }
    
  // is there a file matching cfname?
  if((cfpath = getenv("FOCUS_ROBOTICS_CALIB_DIR"))) {
    sprintf(calibFileName, "%s/%s", cfpath, cfname);
    if(stat(calibFileName, &finfo)==0) {
      // FIXME: should verify that it's a regular file and readable
      printf("FrEagleStereoSmartCam::findCalibFile: Using calibration file %s\n", calibFileName);
      return calibFileName;
    }
  }
  // FIXME: could also check for default.calib in this dir

  // THIRD, if no calib file found look in /etc/fr
  // is there a file matching cfname?
  sprintf(calibFileName, "/etc/fr/%s", cfname);
  if(stat(calibFileName, &finfo)==0) {
    // FIXME: should verify that it's a regular file and readable
    printf("FrEagleStereoSmartCam::findCalibFile: Using calibration file %s\n", calibFileName);
    return calibFileName;
  }
  // is there a default.calib file?
  sprintf(calibFileName, "/etc/fr/default.calib");
  if(stat(calibFileName, &finfo)==0) {
    // FIXME: should verify that it's a regular file and readable
    printf("FrEagleStereoSmartCam::findCalibFile: Using calibration file %s\n", calibFileName);
    return calibFileName;
  }

  // If all else fails, return a null string and the StereoRectMap will use an 
  // identity map by default.
  calibFileName[0] = 0;
  printf("FrEagleStereoSmartCam::findCalibFile: Failed to find a valid calibration file for this camera.\n");
  return calibFileName;
}




/**
 *
 */
int FrEagleStereoSmartCam::getChanForType(FrImageType t) {
  if(chanType[0]==t) return 0;
  else if(chanType[1]==t) return 1;
  else if(chanType[2]==t) return 2;
  else if(chanType[3]==t) return 3;
  else return -1;
}

int FrEagleStereoSmartCam::runPipeOneShot() {return 0;}

int FrEagleStereoSmartCam::initLowLatencyStream() {return 0;}


