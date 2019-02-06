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
 * Creation_Date:  Wed Jan  3 2007
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
#include <stdio.h>  // for printf

#include "FrImage.h"
extern "C" {
#include <pgm.h>
}
#include <assert.h>
#include <math.h>

// Constructors
/**
 * Most commonly used constructor just creates an image of the specified size
 * with undefined data.
 */
FrImage::FrImage(int w, int h, int c) :
  width(w), height(h), nchans(c)
{
  // FIXME: alignment issues for strange widths? Maybe not for uchar data
  // would it be better to have all rows start on 4 or 8 byte alignment?
  // that would normally end up being the case, but should I enforce it?
  stride = width*nchans;
  // allocate image data
  idata = (uchar*)malloc(height*width*nchans);
  idataIsRef = false;
  iplhdr = 0;
}

/**
 * Copy Constructor to create a new FrImage which is an identical copy of an old
 * one.
 */
FrImage::FrImage(const FrImage& img) :
  width(img.width), height(img.height), nchans(img.nchans)
{
  stride = width*nchans;
  // allocate image data
  idata = (uchar*)malloc(height*width*nchans);
  idataIsRef = false;
  iplhdr = 0;
  // copy old image data to new image
  copyInData(img.idata, img.stride);
}


/**
 * Construct from a char buffer (optional data copy)
 */
FrImage::FrImage(uchar* data, int s, int w, int h, int c, int copy) :
  width(w), height(h), nchans(c)
{
  if(copy) {
    // alloc a new buffer and copy data in
    stride = width*nchans;
    idata = (uchar*)malloc(height*width*nchans);
    idataIsRef = false;
    // copy in from data
    copyInData(data, s);
  } else {
    // just ref the data buffer and use it's stride
    stride = s;
    idata = data;
    idataIsRef = true;
  }
  iplhdr = 0;
}

FrImage::~FrImage() {
  if(!idataIsRef) {
    free(idata);
  }
}

// Should fail if requesting pixel outside the image, or if requesting wrong type
// There could also be a flag to just return 0 for anything outside the image
// which is what you want in lots of situations.
/**
 *
 */
void FrImage::setPixel(uchar pix, int x, int y, int chan) {
  // FIXME: range checking should be done, perhaps this should be inline
  // would match stdlib if operator() didn't range check, but get/set pixel did.
  if(chan==-1) {
    for(int i=0; i<nchans; i++) *(idata + y*stride + x*nchans + i) = pix;
  } else {
    *(idata + y*stride + x*nchans + chan) = pix;
  }
}

/**
 *
 */
uchar FrImage::getPixel(int x, int y, int chan) const {
  // FIXME: range checking should be done; perhaps this should be inline
  return *(idata + y*stride + x*nchans + chan);
}


// copy data from a buffer with given stride, image must already be created with
// the correct width, height, type, and number of channels
/**
 *
 */
void FrImage::copyInData(uchar* data, int s) {
  for(int y=0; y<height; y++) {
    uchar* bptr = data + y*s;
    uchar* iptr = getDptr(y);
    for(int x=0; x<(width*nchans); x++) {
      *iptr++ = *bptr++;
    }
  }
}

/**
 *
 */
void FrImage::copyOutData(uchar* data, int s) {
  for(int y=0; y<height; y++) {
    uchar* bptr = data + y*s;
    uchar* iptr = getDptr(y);
    for(int x=0; x<(width*nchans); x++) {
      *bptr++ = *iptr++;
    }
  }
}


// Functions to get image information. Maybe should be declared inline.
int FrImage::getHeight() const {return height;}
int FrImage::getWidth() const {return width;}
int FrImage::getChans() const {return nchans;}
int FrImage::getStride() const {return stride;} // normally not needed, but needed if you grab the Dptr for direct access
uchar* FrImage::getDptr(int row) const {
  if(row) return idata + stride*row;
  else return idata;
}

// Function for interoperability with OpenCV
#ifdef USING_OPENCV
/**
 *
 */
FrImage::FrImage(IplImage* img, int copy) :
  width(img->width), height(img->height), nchans(img->nChannels)
{
  // FIXME: check that type is 8U for IplImage or fail!
  if(copy) {
    // alloc a new buffer and copy data in
    stride = width*nchans;
    idata = (uchar*)malloc(height*width*nchans);
    idataIsRef = false;
    // copy in from data
    copyInData((uchar*)img->imageData, img->widthStep);
  } else {
    // just ref the data buffer and use it's stride
    stride = img->widthStep;
    idata = (uchar*)img->imageData;
    idataIsRef = true;
  }
}

/**
 *
 */
IplImage* FrImage::getIplImage(int copy) {
  IplImage *imgCopy;
  if(copy) {
    // if copy is true, create the full image and return it
    imgCopy = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, nchans);
    copyOutData((uchar*)imgCopy->imageData, imgCopy->widthStep);
    return imgCopy;
  }
  // if copy is false, just make sure the iplhdr matches the current image
  // make the header if it's pointer is null, then copy image size, type, etc. to it
  if(!iplhdr) {
    iplhdr = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, nchans);
    iplhdr->nSize = sizeof(IplImage);
    iplhdr->imageData = (char*)idata;
    iplhdr->widthStep = stride;
  }
  return iplhdr;
}

/**
 *
 */
void FrImage::copyInData(IplImage* img) { // error if sizes don't match

}
#endif

// Functions to support verilog simulations
/**
 *
 */
FrImage::FrImage(std::string filename) {
  FILE *pgmfile;
  gray **pgmdata;
  gray maxval;

  // load image and set params
  printf("FrImage: Loading pgm file %s... ", filename.c_str());
  pgmfile = pm_openr(filename.c_str());
  pgmdata = pgm_readpgm(pgmfile, &width, &height, &maxval);
  pm_close(pgmfile);
  printf("width %d, height %d, and maxval %d\n", width, height, maxval);
  assert(maxval<256);
  assert(width%4==0); // maybe not strictly required, but assures some alignment
  nchans=1; stride=width; // NOTE: fixed settings for monochrome images for now

  // malloc array and copy pgmdata to imageData
  idata = (uchar*)malloc(width*height);
  uchar *d = idata;
  for(int y=0;y<height;y++) {
    for(int x=0;x<width;x++) {
      *d++ = pgmdata[y][x];
    }
  }
  idataIsRef = false;
  iplhdr = 0;
}

/**
 *
 */
void FrImage::saveImage(std::string filename) {
  assert(nchans==1); // save only works for monochrome images right now
  printf("FrImage: Now saving image with width %d and height %d as file %s\n", width, height, filename.c_str());
  gray ** pgmdata = pgm_allocarray(width, height);
  for(int y=0;y<height;y++) {
    for(int x=0;x<width;x++) {
      pgmdata[y][x] = getPixel(x,y);
    }
  }
  FILE *pgmfile = pm_openw(filename.c_str());
  gray maxval = 255; // NOTE: may not always be OK
  pgm_writepgm(pgmfile, pgmdata, width, height, maxval, 0);
  pm_close(pgmfile);

}

/**
 *
 */
void FrImage::setDword(int data, int addr) {
  int x = addr & 0xFF;
  int y = (addr>>8) & 0x3FF;
  int *rowBase = (int*)(idata + y*stride);
  rowBase[x] = data;
}

/**
 *
 */
int FrImage::getDword(int addr) {
  int x = addr & 0xFF;
  int y = (addr>>8) & 0x3FF;
  int *rowBase = (int*)(idata + y*stride);
  if(x<width && y<height) {
    return rowBase[x];
  } else {
    // FIXME: might want to optionally warn if reading outside of the image
    return 0;
  }
}

/**
 *
 */
void FrImage::setGradientImage(uchar startVal, float hrate, float vrate) {
  float rowStartVal = startVal;
  uchar* pptr = idata;
  for(int y=0;y<height;y++) {
    // FIXME: assumes stride==width which isn't always the case though it often is
    float curVal = rowStartVal;
    for(int x=0;x<width;x++) {
      *pptr++ = (uchar)fabsf(curVal);
      curVal += hrate;
    }
    rowStartVal += vrate;
  }
}

/**
 *
 */
int FrImage::isEqual(const FrImage& img, int verbose) {
  int total_miscompares = 0;
  for(int y=0; y<height; y++) {
    for(int x=0; x<width; x++) {
      if(getPixel(x,y) != img.getPixel(x,y)) {
	printf("Image miscompare at x=%d, y=%d; src=%d, dst=%d\n", x, y, getPixel(x,y), img.getPixel(x,y));
	total_miscompares++;
      }
    }
  }
  return total_miscompares;
}

/**
 * This flavor of operator= can copy an arbitrary FrImage to another one with
 * the same width, height, and nchans. It can also copy a 3 channel image to a
 * single channel or a single channel to a 3 channel of the same width and
 * height.
 */
void FrImage::operator=(const FrImage& src) {
  // if width, height, nchans all the same then direct copy
  // if width and height the same but src 1 chan and dst 3 chan then replicate grey values to all three channels
  // if width and height the same but src 3 chan and dst 1 chan then convert rgb to grey
  if(width==src.width && height==src.height && nchans==src.nchans) {
    copyInData(src.idata, src.stride);
  } else if(width==src.width && height==src.height && nchans==3 && src.nchans==1) {
    for(int y=0; y<height; y++) {
      uchar* sptr = src.getDptr(y);
      uchar* dptr = getDptr(y);
      for(int x=0; x<width; x++) {
	*dptr++ = *sptr;
	*dptr++ = *sptr;
	*dptr++ = *sptr++;
      }
    }
  } else {
    // ERROR CASE: throw an exception, leave images unchanged
    // FIXME: at least print a message initially
    printf("ERROR FrImage::operator= Attempted assignment of different size images.\n");
  }
}


/**
 * This flavor of operator= sets all pixels in the image to the value provided.
 * It is easy to clear a gray or color image to white or black or any shade of
 * gray, but not to clear a color image to a color. I could pass in an int to
 * set up to 4 channels in a color image. I could also provide another flavor of
 * operator= that accepts a helper class called FrPixelColor or FrImageColor or
 * somesuch. Color could also be arbitrarily specified as a 1x1 image, but that
 * seems weird and would be confusing with copying the whole image where sizes
 * must match.
 */
void FrImage::operator=(const uchar val) {
  for(int y=0; y<height; y++) {
    uchar* dptr = getDptr(y);
    for(int x=0; x<(width*nchans); x++) {
      *dptr++ = val;
    }
  }  
}






