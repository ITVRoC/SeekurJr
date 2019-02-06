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
 * Creation_Date:  Sun Apr  1 2007
 * 
 * Brief Description:
 * This class provides a high-level interface to the Eagle device (nDepth vision
 * processor)
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

#ifndef FREAGLESTEREOSMARTCAM_H
#define FREAGLESTEREOSMARTCAM_H
#include <stdio.h>
#include "FrImage.h"
#include "FrOptArgs.h"
#include "FrEagleDevice.h"
#include "FrStereoRectMap.h"
#include "FrEagleStereoCam.h"

/**
 * This class provides a control and data interface to an nDepth vision system.
 * The id parameter to the constructor specifies which Eagle device to connect
 * to if multiple devices are present in the system. Devices are numbered 
 * according to their PCI bus and slot. See the EagleDevice class for details.
 */
class FrEagleStereoSmartCam {
 public:
  /**
   * Constructors and Destructor
   * 
   * The primary constructor is based only on camera id. An alternate constructor
   * takes a pointer to an FrOptArgs object to support writing programs with a
   * common set of arguments which control the camera's behavior.
   * I don't expect to directly support the fr3 v4l driver with this class.
   */
  FrEagleStereoSmartCam(int id=0);
  FrEagleStereoSmartCam(FrOptArgs *a);
  ~FrEagleStereoSmartCam();


  /**
   * Stereo Camera Properties
   *
   * Get/Set resolution, framerate, pixel format, etc
   * The issue is that resolution and pixel format may affect other subblocks.
   * Perhaps resolutions should only be handled at the overall smartCam level
   * since I really can't envision a need to set camera resolution to something
   * different than resolution of other procs. I can imagine different modes for
   * downsampling camera data, however and that may need to be added some time.
   *
   * Get/Set brightness (actual and desired), exposure, gain, aegc mode, aegc properties
   * FIXME: all camera image properties should be independantly settable for both sensors
   * What about initializing or reinitializing the camera? How about changing
   * resolution on the fly? Doesn't resolutions apply to other procs as well?
   * I.e., isn't resolution more of a global properly for the whole ssc?
   *
   * Getting and setting raw I2C regs is a hack that makes the interface less
   * general, but it will be left in the short term to support the C API.
   *
   * The original plan on camera settings was to be able to set a select for which
   * camera would be being adjusted, and then adjust that camera. Change the sel
   * when you need to deal with a new camera. That gets annoying in practice since
   * you have this extra call every time the selected sensor changes and code is
   * less readable since you must track extra state to know what camera you are 
   * operating on. Probably just having a select for every function which defaults
   * to 'all' is the right way to go.
   */
  // Frame rate may just be a camera thing. Other blocks really have no concept
  // of frame rate and just go as fast as they can.
  // float, or fps*1000; return actual fps set; could also set ms/frame
  int setFrameRate(int fps) {return ecam->setFrameRate(fps);}

  void setCamBrightness(int b); // turn on AEGC, set desired bin (maybe scaled)
  void setDesiredBin(int b, int sel=-1); // bins 1 to 64, will I ever want to set left and right differently?
  int getDesiredBin(int sel);
  int getCurrentBin(int sel);
  //void setBrightness(int b, int sel); // brightness is 0 to 255, sel is -1,0,1
  //int getBrightness(int sel);

  void setCamExposure(int e); // turn off AEGC, set exposure value
  int getCurrentExp(int sel);
  void setExp(int e, int sel=-1);
  int getExp(int sel);
  //void setExposure(int e, int sel); // what is the units of e?
  //int getExposure(int sel);
  void setCamGain(int g); // turn off AEGC, set gain value
  int getCurrentGain(int sel);
  void setAGain(int g, int sel=-1);
  int getAGain(int sel);
  //void setGain(int g, int sel); // perhaps gain should include both digital and analog gain
  //int getGain(int sel);
  //void enableDigitalGain(); // enable the use of digital gain
  // if setGain includes digital gain, there will have to be an approximately linear
  // scale from top to bottom. Should it be in dB?

  // Automatic exposure and gain control can be set to work multiple ways.
  // 0: it can be controlled manually. Setting raw exp or gain assumes you entered this mode.
  // 1: it can be set to use the internal sensor AEC and AGC
  // 2: the verilog aegc circuit in the FPGA can be used
  // 3: the software algorithm can be used
  // 4: alternate software algo which uses digital gain
  // where does high dynamic range mode come in here?
  // 0=manual, 1=sensor(unlocked), 2=fpga(locked), 3=sw(locked)
  void setAegcMode(int mode);
  int getAegcMode() {return aegcMode;}
  void setAegcSkip(int frames) {aegcSkip = frames; edev->setCSR(EAGLE_CSR_AEGC_SKIP_FRAMES, frames);}
  int getAegcSkip() {return aegcSkip;}

  // set up both image sensors to be ready for a AEGC ROI. This includes adjusting
  // tile coordinates and sample weight for all 25 regions. The middle region (2,2)
  // will be the only one with any weight for AEGC
  void initAegcRoi() {ecam->initAegcRoi();}
  // set a region to be the only one considered by the AEGC algorithm
  // This will basically set regs 155, 156, 161, 162
  void setAegcRoi(int x1, int y1, int x2, int y2, int sel) {ecam->setAegcRoi(x1,y1,x2,y2,sel);}

  // raw i2c access, for now at least
  void setI2cReg(int offset, int dev, int value);
  int getI2cReg(int offset, int dev);

  /**
   * Rectification Processor Properties
   *
   * Changing rect map in an active camera useful for rectchecking stream during
   * calib. Get is just there to keep things symmetric. Setting raw data is 
   * really not the right thing to do since it would lead to incorrect values
   * returned for baseline and focallength, but is useful for things to work well
   * with the C API for now.
   * Need to get baseline and focallength to measure distance, though that might
   * be more of a property of the stereo camera. Leave in this section for now.
   */
  void setRectMap(FrStereoRectMap *r);
  FrStereoRectMap *getRectMap();
  void setRectData(int* data, int len); // FIXME: this would break return values for baseline and focallength!
  // FIXME: only return baseline/focallength from the rmap if the rmap is valid!
  float getCameraBaseline() {return rmap->getCameraBaseline();}
  float getCameraFocalLength() {return rmap->getCameraFocalLength();}

  /**
   * Correlation Processor Properties
   *
   * There isn't a lot here right now, but hopefully there will continuously be
   * more added. In additions to setting min and max disp search range, trunc
   * value should be settable, window size should be settable, left/right check
   * should be settable, and other pre and post processing and checks that might
   * be added should be settable.
   */
  void setMatchQualThresh(float t); // range: 0->256
  void setMatchQualThresh(int t); // range: 0->127
  int getMatchQualThresh() {return matchQualThresh;}
  void setDispSearchRange(int r);


  /**
   * Streaming Data Controller Properties
   * This block controls data exchange between the other procs and between other
   * procs and the host. Individual frames can be moved for specific processing
   * and streams can be set up. Low latency and full frame buffering are
   * controlled. Transfers both to the host and to external ports via the vtx
   * are controlled
   */
  // get/setImg is a straight copy to or from the hardware
  // FIXME: not yet implemented!!!
  void setImg(FrImageType type, FrImage img);
  FrImage *getImg(FrImageType type);

  // Set up a transfer of a particular image type on a particular channel
  // Channels are set to transfer image type "none" by default
  // function could fail if passed an illegal channel or type and should return
  // an int.
  void setChanType(int c, FrImageType t);
  // Initiate a transfer for a particular channel or type; FIXME: using type should be removed!
  // If passed chan=-1 (the default) grab all channels of type!=none
  void grabImg(FrImageType type); // grab tells the hw to cycle
  void grabImg(int chan=-1); // grab tells the hw to cycle
  // Actually return the previously grabbed image. Will block if transfer not complete
  // retrieve may overwrite the previous img since it provides storage. 
  FrImage *retrieveImg(FrImageType type); 
  FrImage *retrieveImg(int chan); 

  // vtx: 0 for no trans, 1 to enable transfer
  // Always transfers right rect and disp for now, add controls for more flexibility
  // Devsys controls should really be in a derived class since devsys is a superset of fr3
  // This call should be renamed to something more intuitive.
  void setDevsysMode(int mode) {vtxTransferEnable = mode;}

  /**
   * Overall Properties and Operations
   *
   * Frame size, frame rate, processing mode, etc. apply to the whole system and
   * each of the components. They could be set in each subsection or just once 
   * for the whole processor.
   *
   * Run() is the major overall operation. Other operations might include reset
   * and re-initialization.
   *
   */
  int getChipVersion() {return edev->getCSR(EAGLE_CSR_CHIP_VERSION);}

  // test&debug: set 0 to not enable or wait for procs, 1 for rect, 2 for stereo, 3 for both
  void setProcEnables(int mode) {rectProcEnable = mode&0x01; corrProcEnable = mode & 0x02;}

  // Modes are one-shot or streaming and pipelined or low latency
  // Right now low latency isn't implemented in the hardware, so everything is pipelined
  // I have only supported streaming thus far, but one-shot is probably better for low frame rates
  // all four cases should eventually be supported, though you probably won't use pipelined
  // when low latency becomes available. Should take an enum.
  // This call should be renamed to something more intuitive.
  void setMode(int streaming, int lowlatency);

  // return actual width and height (how?) or error if requested size is unsupported
  void setFrameWidth(int w) { frameWidth = w; }
  int getFrameWidth() { return frameWidth; }
  void setFrameHeight(int h) { frameHeight = h; }
  int getFrameHeight() { return frameHeight; }


  // Run comes from the rect proc and stereo proc baseclasses. It should run those
  // procs internally, though the call might be implicit in a streaming situation.
  int run();


  // Protected and Private methods and variables not part of the public interface
 protected:
  void runSoftAEGC();
  void calcNextParmsAEGC();
  void calcNextParmsAEGC2();
  void runIndSoftAegc();

  int initHWFrameList();
  int stepHWFrameList();
  int getActiveHWFrame(FrImageType t);
  int getCompleteHWFrame(FrImageType t);

  void initProcPipe();
  void stepProcPipe();
  void waitHWIdle();

  void startRectProc();
  void waitRectDone();
  void startCorrProc();
  void waitCorrDone();
  void waitProcsDone();

  // Returns filename for calib file, or special name for identity (null?) if none found
  const char* findCalibFile();

  int getChanForType(FrImageType t);
  int runPipeOneShot(); // this could be done low-latency or pipelined. the only difference would be latency and max frame rate
  int initLowLatencyStream(); // the low latency stream would be the best, but how to sync?


 private:
  static const int aegcVerbose = 0;
  FrEagleDevice *edev;
  FrStereoRectMap *rmap;
  FrEagleStereoCam *ecam;
  FrOptArgs *args;

  // map types to img locations in hw
  // need maps for current processing (what is currently progammed into hw and running)
  // need maps for last processing (what just finished processing and can be DMA'd)
  // what I really want is an array of int selected by image type enumerated type
  // curr_srcs[ITYPE_RIGHT_RECT], prev_srcs[ITYPE_RIGHT_RAW]
  // A function getActiveHWFrame(type) and getCompleteHWFrame(type) and stepHWFrames() and initHWFrames()
  int activeHWFrames[5];
  int completeHWFrames[5];

  FrImageType chanType[4];
  FrImage *chanImg[4];
  char calibFileName[256];
  int vtxTransferEnable;
  int rectProcEnable;
  int corrProcEnable;

  int aegcMode;
  int aegcSkip;

  int reqBin, reqBinR, reqBinL;
  int curBinR, curBinL;
  int curExp, curExpR, curExpL;
  int curGain, curGainR, curGainL;
  int newExp, newExpR, newExpL;
  int newGain, newAGainR, newAGainL;

  int curFrameNum;
  int matchQualThresh;
  int frameWidth;
  int frameHeight;
};

#endif


