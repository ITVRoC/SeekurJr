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
 * Creation_Date:  Thu May  3 2007
 * 
 * Brief Description:
 * 
 * Functionality:
 * 
 * Issues:
 * 
 * Limitations:
 *  Add ability to remap to float to support conversion from disp to Z.
 * 
 * Testing:
 * 
 ******************************************************************************/

#include "FrImageLUTProc.h"

/**
 *
 */
FrImageLUTProc::FrImageLUTProc(FrImageLUTType type, FrImage *src, int min, int max) {
  dstImg = 0;
  lut0 = 0;
  lut1 = 0;
  lut2 = 0;
  lutType = type;
  srcImg = src;
  srcMin = min;
  srcMax = max;
}


/**
 *
 */
FrImageLUTProc::~FrImageLUTProc() {
  delete(dstImg);
  delete(lut0);
  delete(lut1);
  delete(lut2);
}


/**
 * Change the LUT Type
 * Since the LUTs are created automatically for the type which is set if their 
 * pointers are null, this is just a matter of freeing everything that has been
 * set up, setting pointers to null, setting the new LUT type and running. 
 * Everything will be recreated as necessary.
 */
void FrImageLUTProc::setLUTType(FrImageLUTType type) {
  delete(dstImg);
  delete(lut0);
  delete(lut1);
  delete(lut2);
  dstImg = 0;
  lut0 = 0;
  lut1 = 0;
  lut2 = 0;
  lutType = type;  
}


/**
 *
 */
void FrImageLUTProc::setSrcMaxVal(int val) {
  srcMax = val;
}


/**
 *
 */
void FrImageLUTProc::setImg(FrImageType type, FrImage *img) {
  srcImg = img;
}


/**
 *
 */
FrImage* FrImageLUTProc::getImg(FrImageType type) {
  return dstImg;
}


/**
 *
 */
int FrImageLUTProc::run() {
  // (re)allocate dstImg if necessary
  // FIXME: what about custom LUT types? Not yet supported.
  // If the LUT size or type changes, just free everything and set pointers to
  // null. It will all be automatically reallocated here.
  if(dstImg==0) {
    int dstChans = (lutType==LUT_TYPE_FULL_RANGE) ? 1 : 3;
    dstImg = new FrImage(srcImg->getWidth(), srcImg->getHeight(), dstChans);
  }
  // create LUT if necessary
  if(lut0==0) {
    lut0 = new uchar[256];
    lut1 = new uchar[256];
    lut2 = new uchar[256];
    if(lutType==LUT_TYPE_FULL_RANGE) {
      createFullRangeLUT();
    } else {
      createColorSpectrumLUT();
    }
  }
  // call appropriate remap function
  if(lutType==LUT_TYPE_FULL_RANGE) {
    remapGray();
  } else {
    remapColor();
  }
  return 0;
}


/**
 * Creates lut0 such that the range srcMin to srcMax will be remapped to 0...255
 */
void FrImageLUTProc::createFullRangeLUT() {
  // depends on srcMin and srcMax; srcMin will produce 0 and srcMax will produce
  // 255. The output range isn't selectable. Any input less than srcMin or more
  // than srcMax is an error, but will just be mapped to zero or 255 in the LUT.
  for(int i=0; i<srcMin; i++) lut0[i] = 0;
  for(int i=srcMax; i<256; i++) lut0[i]=255;
  int levels = (srcMax - srcMin) + 1;
  float step = 256.0f/levels;
  for(int i=0; i<levels; i++) lut0[i+srcMin] = (int)(i*step);
}


/**
 * Creates luts 0,1,2 such that range srcMin to srcMax will be remapped from
 * yellow for srcMax through orange, red, and blue for srcMin.
 * FIXME: assumes srcMin is always zero!
 */
void FrImageLUTProc::createColorSpectrumLUT() {
  for(int disparity=0; disparity<srcMax; disparity++) {
    if ( disparity == 0 ) {
      lut0[disparity] = 0; // red
      lut1[disparity] = 0; // green
      lut2[disparity] = 0; // blue
    } else if ( disparity > (srcMax/2) ) {
      lut0[disparity] = 255; // red
      lut1[disparity] = (255 - 2*(255-(disparity*(256/(srcMax+1))))); // green
      lut2[disparity] = 0;   // blue	      
    } else {
      lut0[disparity] = (255- 2*(255-(disparity*256/(srcMax+1)))); // red
      lut1[disparity] = 0; // green
      lut2[disparity] =  (2*(255-(disparity*256/(srcMax+1))));  // blue	      
    }
    //printf("disp=%d, red=%d green=%d blue=%d\n", disparity, redLut[disparity], greenLut[disparity], blueLut[disparity]);
  }

}


/**
 * Creates dst from src using lut0, lut1, lut2 for channels 0, 1, and 2 in dst.
 * dst MUST be a three channel image.
 */
void FrImageLUTProc::remapColor() {
  for(int y=0; y<srcImg->getHeight(); y++) {
    uchar *srcPtr = srcImg->getDptr(y);
    uchar *dstPtr = dstImg->getDptr(y);
    for(int x=0; x<srcImg->getWidth(); x++) {
      *dstPtr++ = lut2[*srcPtr];
      *dstPtr++ = lut1[*srcPtr];
      *dstPtr++ = lut0[*srcPtr++];
    }
  }
}


/**
 * Creates dst from src using lut0. dst MUST be a one channel image.
 */
void FrImageLUTProc::remapGray() {
  for(int y=0; y<srcImg->getHeight(); y++) {
    uchar *srcPtr = srcImg->getDptr(y);
    uchar *dstPtr = dstImg->getDptr(y);
    for(int x=0; x<srcImg->getWidth(); x++) {
      *dstPtr++ = lut0[*srcPtr++];
    }
  }
}




