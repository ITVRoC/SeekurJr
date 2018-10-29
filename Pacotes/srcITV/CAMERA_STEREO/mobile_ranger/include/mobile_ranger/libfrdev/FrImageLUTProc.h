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
 * Creation_Date:  Wed May  2 2007
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
#ifndef FRIMAGELUTPROC_H
#define FRIMAGELUTPROC_H

#include "FrMsgLog.h"
#include "FrImage.h"

enum FrImageLUTType { LUT_TYPE_CUSTOM, LUT_TYPE_FULL_RANGE, LUT_TYPE_COLOR_SPECTRUM };

/// This class transforms an image by changing each pixel with an arbitrary LUT.
/**
 * This class transforms a source image into a destination image by converting
 * every pixel with an arbitrary look-up table. There will be a few standard
 * LUTs defined that can just be selected and also a way to define a new
 * arbitrary mapping by specifying the output for every input.
 *
 * Initially, the only source image type supported will be single channel
 * FrImage. The output will be 1 channel or 3 channel FrImage. Other
 * combinations may be added in the future.
 *
 * Standard LUTs will be the full range expansion which maps an image with a
 * small range (i.e. 0 to 63) to the full range of 0 to 255 for better viewing
 * and the color spectrum which maps the largest input values to hot colors and
 * the lowest input values to cool colors.
 */
class FrImageLUTProc {
 public:
  FrImageLUTProc(FrImageLUTType type, FrImage *src, int min, int max);
  ~FrImageLUTProc();

  void setLUTType(FrImageLUTType type);
  void setSrcMaxVal(int val); // just to make things easier for now
  void setImg(FrImageType type, FrImage *img);
  FrImage* getImg(FrImageType type);
  // Run should allocate the dst image and create the LUT. Additional runs
  // should only recreate the LUT if params are changed. The user can normally
  // just construct the LUT obj and then call run() and getImg() for each new
  // frame. Setting the range or max value after construction is also necessary
  // for now, but that could be determined automatically in the future, maybe.
  int run();

 protected:
  void createFullRangeLUT();
  void createColorSpectrumLUT();
  void remapColor();
  void remapGray();

 private:
  uchar *lut0;
  uchar *lut1;
  uchar *lut2;
  FrImage *srcImg;
  FrImage *dstImg;
  FrImageLUTType lutType;
  int srcMax, srcMin;
};
#endif
