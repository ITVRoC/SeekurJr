/****************************************************************************
 * Copyright (c) 2006 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Tue Dec 12 2006
 * 
 * Brief Description:
 * Basic image class used in all Focus Robotics software.
 * 
 * Functionality:
 * This class always holds an uncompressed image, either grayscale or color.
 * It might also contain similar data that can be viewed as an image, such as
 * disparity data or quality data;
 * 
 * Issues:
 * Consider adding basic operators like +,-,|,&,!
 * This type could also be a matrix--add matrix operation support
 * It seems like there should be a FrImageFile class to handle loading and
 *  saving all types of images and formats; this would match FrVideoFile, but 
 *  should there be an FrVideo for image sequences just stored in memory??
 * Sometimes, classes like this one that must work with an aggragation of lots
 *  of different elements are created as template classes. Would this be better
 *  implemented as a template class?
 * How about an FrCompressedImage class? That would support compressed image
 *  formats in memory. Would that be useful?
 * Should I add a region of interest and channel of interest to fully support
 *  opencv/iplimage functionality?
 * FrImage will be the base point for interoperability with other libraries. It
 *  will have a few ifdef's like ifdef OPENCV and LTILIB to enable support for
 *  conversion with that libraries image data type.
 * The implementation doesn't ensure alignment right now, so widths probably
 *  have to be a multiple of 4 or 8. I don't enforce it, but it's probably 
 *  required.
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/
#ifndef FRIMAGE_H
#define FRIMAGE_H

#define USING_OPENCV
#ifdef USING_OPENCV
#include "cxcore.h"
#endif

// This is in opencv types, but I want to keep it when we don't use opencv
typedef unsigned char uchar;
#include <stdlib.h>
#include <string>

// NOTE: I'm trying this image type stuff out. It may be too much of a pain because there
// turns out to be a zillion types. A global type makes it easier to combine some
// procs, though.
/// Enum of all image types identified as input or output to the system.
enum FrImageType { ITYPE_DEFAULT, ITYPE_RAW, ITYPE_SMOOTHED, ITYPE_EDGE,
		   ITYPE_RIGHT_RAW, ITYPE_LEFT_RAW, 
		   ITYPE_RIGHT_RECT_SRC, ITYPE_LEFT_RECT_SRC, ITYPE_RIGHT_RECT, ITYPE_LEFT_RECT,
		   ITYPE_RREF_DISP, ITYPE_LREF_DISP, ITYPE_FINAL_DISP,
		   ITYPE_DISP_SUBPIX, 
		   ITYPE_MQUAL_MASK, ITYPE_TEXT_MASK, ITYPE_SURF_MASK, ITYPE_LRCHECK_MASK,
		   ITYPE_ANALYZER_RRECT, ITYPE_ANALYZER_LRECT, ITYPE_ANALYZER_DISP, ITYPE_ANALYZER_CGRAPH,
		   ITYPE_REMAPPED_LUT, ITYPE_FILTERED, ITYPE_FILTER_SRC,
		   ITYPE_ALL, ITYPE_NONE, ITYPE_OTHER };
/// Basic image class used throughout the library.
/**
 * This class represents a flexible image container which is used by all the
 * image generation and manipulation routines in libfrdev. It can represent a
 * single channel (grayscale) or multi-channel (color) image of any size. It
 * also provides easy ways to interact with other image types from other
 * libraries providing a bridge between libfrdev and other computer vision
 * libraries.
 */
class FrImage {
 public:
  // Constructors
  /// Basic constructor, pass height, width, number of channels
  FrImage(int w, int h, int c);
  /// Copy constructor
  FrImage(const FrImage& img);
  /// Construct from a char buffer with stride s. Data can be copied or referenced.
  // Data is copied by default, set copy=0 to reference data
  // Data is assumed to be interleaved. Should I also support planar?
  FrImage(uchar* data, int s, int w, int h, int c, int copy=1);
  ~FrImage();

  // Access functions, these need to be overloaded for various possible data types
  // should fail if requesting pixel outside the image, or if requesting wrong type
  // for image ptype. Should overload for 1, 4, 8 byte unsigned, signed, float at least.
  void setPixel(uchar pix, int x, int y, int chan=0);
  uchar getPixel(int x, int y, int chan=0) const;

  // copy data from a buffer with given stride, image must already be created with
  // the correct width, height, type, and number of channels. This could be
  // done with operator=, but we wouldn't know the data buffer stride.
  void copyInData(uchar* data, int s);
  void copyOutData(uchar* data, int s);

  // Functions to get image information. This data can't be set after the image 
  // is allocated so there are no corresponding set functions.
  // A data array can be reshaped in some cases, though, so the reshape or setROI
  // functions might change what is returned here.
  int getHeight() const;
  int getWidth() const;
  int getChans() const;
  int getStride() const; // normally not needed, but needed if you grab the Dptr for direct access
  uchar* getDptr(int row=0) const;

  // Functions for interoperability with OpenCV
#ifdef USING_OPENCV
  // Data is copied by default, set copy to 0 to only reference the img
  // Error if IplImage is other then type 8U
  // Constructor
  FrImage(IplImage* img, int copy=1);
  // Converter
  IplImage* getIplImage(int copy=1); // if copy=0 just ref the same image data
  // Assignment in
  // FIXME: This is maybe better done with operator=
  void copyInData(IplImage* img); // error if sizes don't match or img not type 8U
  // Should I add assignment out?
#endif // ifdef USING_OPENCV

  // Functions used for verilog simulations--may also be useful for other cases
  // Construct from a pgm or save to a pgm
  FrImage(std::string filename);
  void saveImage(std::string filename);
  // get or set a dword with harware addressing scheme
  void setDword(int data, int addr);
  int getDword(int addr);
  // create test images
  void setGradientImage(uchar startVal, float hrate, float vrate);
  // test equality; this should be expanded to generate diff images, check max diffs, etc.
  // operator== could be created trivially from this function, diff verbosity could be a flag
  int isEqual(const FrImage& img, int verbose=0);

  void operator=(const FrImage& src);
  void operator=(const uchar val);

  // Lots of utilty operations can be added over time
  // diff(FrImage&) to create a difference image: the abs val of a subtraction
  // clear() to set all pixels to zero
  // set(uchar) to set all pixels to the given value
  // operator=() for assignment from another FrImage and many other types
  // operator+() for accumulation
  // operator&() for masking or bit rops
  // operator*() for scaling
  // a way to do concatenation to make larger images from multiple smaller ones

 protected:
  int width;      // Width in pixels
  int height;     // Height in pixels
  int stride;     // Distance between consecutive rows in bytes
  int nchans;     // Number of channels in the image--essentially the size of the 3rd dimentions
                  // what about support for n dimensional arrays?
  uchar* idata;    // Pointer to the actual image data
  bool idataIsRef;// Set if we reference image data rather then alloc it in the constructor
#ifdef USING_OPENCV
  IplImage* iplhdr;
#else
  void* iplhdr;
#endif
};

#endif
