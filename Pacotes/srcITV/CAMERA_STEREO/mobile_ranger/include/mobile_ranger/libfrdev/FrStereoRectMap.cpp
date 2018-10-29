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
 * Creation_Date:  Fri Jan  5 2007
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
#include <stdio.h>
#include <assert.h>
#include "FrStereoRectMap.h"

/*******************************************************************************
 * Constructors and Destructors
 *
 */
FrStereoRectMap::FrStereoRectMap(int w, int h) : width(w), height(h) {
  // Initialize members
  paramsValid = false;
  mapsValid = false;
  rightXMap = 0;
  rightYMap = 0;
  leftXMap = 0;
  leftYMap = 0;
  hw_remap_coords = 0;

  // FIXME: depreciated members->get rid of them ASAP!
  //rightMap = 0;
  //leftMap = 0;

  // Set parameters to identity
  setIdentityParams();
}

FrStereoRectMap::FrStereoRectMap(const char *filename) {
  // Initialize members
  width = 752;
  height = 480;
  paramsValid = false;
  mapsValid = false;
  rightXMap = 0;
  rightYMap = 0;
  leftXMap = 0;
  leftYMap = 0;
  hw_remap_coords = 0;

  // FIXME: depreciated members->get rid of them ASAP!
  //rightMap = 0;
  //leftMap = 0;

  // Read in camera params file
  // FIXME: should I also allow a format to be passed in?
  loadCameraParams(filename);
}

FrStereoRectMap::~FrStereoRectMap() {
  if(rightXMap) delete(rightXMap);
  if(rightYMap) delete(rightYMap);
  if(leftXMap) delete(leftXMap);
  if(leftYMap) delete(leftYMap);
  if(hw_remap_coords) free(hw_remap_coords);
}


/*******************************************************************************
 * Load, Save, Set, Get, Print calib parameters
 *
 */
// Set everything at once
void FrStereoRectMap::setCameraParams(float *A0, float *k0, float *T0, 
				      float *A1, float *k1, float *T1, 
				      float *Anew, float Baseline) {
  // Copy all values, never just store the pointers passed to us!
  for(int i=0;i<9;i++) {
    Ar[i]=A0[i];
    Al[i]=A1[i];
    Tr[i]=T0[i];
    Tl[i]=T1[i];
    An[i]=Anew[i];
  }
  for(int i=0;i<4;i++) {
    kr[i]=k0[i];
    kl[i]=k1[i];
  }
  B = Baseline;
  paramsValid = true;
  createMaps();
}

// Set safe values with no image transformation
void FrStereoRectMap::setIdentityParams() {
  for(int i=0;i<9;i++) {
    Ar[i]=0;
    Al[i]=0;
    Tr[i]=0;
    Tl[i]=0;
    An[i]=0;
  }
  Ar[0] = 1.0; Ar[4] = 1.0; Ar[2] = width/2; Ar[5] = height/2;
  Al[0] = 1.0; Al[4] = 1.0; Al[2] = width/2; Al[5] = height/2;
  An[0] = 1.0; An[4] = 1.0; An[2] = width/2; An[5] = height/2;
  Tr[0] = 1.0; Tr[4] = 1.0; Tr[8] = 1.0;
  Tl[0] = 1.0; Tl[4] = 1.0; Tl[8] = 1.0;
  for(int i=0;i<4;i++) {
    kr[i]=0;
    kl[i]=0;
  }
  B = 1.0;
  paramsValid = true;
  createMaps();
}

int FrStereoRectMap::saveCameraParams( const char* filename, int fmt )
{
  // only save parameters if they've been set or loaded--how to tell?
  assert(paramsValid==true);
  int i;

  if(fmt==0) {
    // opencv format was:
    // line1: 2 (the number of cameras)
    // line3: camera0 params (27 floats) size(2), A(9), K(4), rotmatr(9), transvect(3)
    // line5: camera1 params (27 floats)
    // line7: quad0 (8) (really 4 pairs=four corner coords)
    // line8: quad1 (8)
    // line9: right transform(9)
    // line10: left transform(9)
    // NOTE: no new intrinsics, no baseline
    float params[88];
    for(i=0;i<88;i++) params[i]=0.0;
    params[0] = params[27] = width;
    params[1] = params[28] = height;
    for(i=0;i<9;i++) {
      params[2+i] = Ar[i];
      params[29+i] = Al[i];
      params[70+i] = Tr[i];
      params[79+i] = Tl[i];
    }
    for(i=0;i<4;i++) {
      params[11+i] = kr[i];
      params[38+i] = kl[i];
    }
    
    FILE* paramsFile = fopen(filename, "w");
    assert(paramsFile);
    
    fprintf(paramsFile, "2\n");
    for(i=0;i<88;i++) {
      fprintf(paramsFile, "%15.10f ", params[i]);
    }
    fprintf(paramsFile, "\n");
    fclose(paramsFile);

  } else if(fmt==1) {
  // fr format would be:
  // line 1: frcalibparamsfileversion 1
  // line 2: imagewidth(pix), imageheight(pix), baseline(mm) (3)
  // line 3: new intrinsics (9)
  // line 4: right intrinsics (9)
  // line 5: right distortion (4)
  // line 6: right transform (9)
  // line 7: left intrinsics (9)
  // line 8: left distortion (4)
  // line 9: left transform (9)
    FILE* paramsFile = fopen(filename, "w");
    assert(paramsFile);
    
    fprintf(paramsFile, "frcalibparamsfileversion 1\n");
    fprintf(paramsFile, "%d %d %f\n", width, height, B);
    for(i=0;i<9;i++) { fprintf(paramsFile, "%15.10f ", An[i]); }
    fprintf(paramsFile, "\n");

    for(i=0;i<9;i++) { fprintf(paramsFile, "%15.10f ", Ar[i]); }
    fprintf(paramsFile, "\n");
    for(i=0;i<4;i++) { fprintf(paramsFile, "%15.10f ", kr[i]); }
    fprintf(paramsFile, "\n");
    for(i=0;i<9;i++) { fprintf(paramsFile, "%15.10f ", Tr[i]); }
    fprintf(paramsFile, "\n");

    for(i=0;i<9;i++) { fprintf(paramsFile, "%15.10f ", Al[i]); }
    fprintf(paramsFile, "\n");
    for(i=0;i<4;i++) { fprintf(paramsFile, "%15.10f ", kl[i]); }
    fprintf(paramsFile, "\n");
    for(i=0;i<9;i++) { fprintf(paramsFile, "%15.10f ", Tl[i]); }
    fprintf(paramsFile, "\n");
    fclose(paramsFile);

  } else {
    printf("Unknown/unsupported format specified to BaseStereoCamera::saveCalibParameters\n");
    return 1;
  }

  return 0; // would only fail if no calib parameters are set or if file isn't accessible
}

int FrStereoRectMap::loadCameraParams( const char* filename, int fmt ) {
  // support loading older version 2 files (which don't include baseline or new intrinsics)
  // as well as the new format I'm creating which will start with other magic: frcalibparamsfileversion 1
  FILE* paramsFile = fopen(filename, "r");

  // If the file doesn't exist or can't be opened, set parameters to identity
  // and print a warning. FIXME:
  assert(paramsFile);

  // Read first line into a string to determine file type
  // Call sub-funtion to load old-style opencv params files
  char line[256];
  fgets(line, 255, paramsFile);
  if(fmt==-1) {
    if(line[0]=='2') {
      fmt=0;
    } else if(sscanf(line, "frcalibparamsfileversion %d", &fmt)!=1) {
      printf("Unable to determine calib file format for file %s in BaseStereoCamera::loadCalibParameters\n", filename);
      fclose(paramsFile);
      return 1;
    }
  }

  // FIXME: check return code and fail if no more params read; maybe just add '|| return 2' for each fscanf
  int i;
  if(fmt==0) {
    float params[88];
    for(i=0;i<88;i++) { fscanf(paramsFile, "%f", &params[i]); }
    // Now put all that data into our internal structures
    width = (int)params[0];
    height = (int)params[1];
    for(i=0;i<9;i++) { Ar[i] = params[2+i]; }
    for(i=0;i<9;i++) { Al[i] = params[29+i]; }
    for(i=0;i<9;i++) { Tr[i] = params[70+i]; }
    for(i=0;i<9;i++) { Tl[i] = params[79+i]; }
    for(i=0;i<4;i++) { kr[i] = params[11+i]; }
    for(i=0;i<4;i++) { kl[i] = params[38+i]; }
    fclose(paramsFile);

  } else if(fmt==1) {
    fscanf(paramsFile, "%d %d %f\n", &width, &height, &B);
    for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &An[i]); }
    for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &Ar[i]); }
    for(i=0;i<4;i++) { fscanf(paramsFile, "%f ", &kr[i]); }
    for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &Tr[i]); }
    for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &Al[i]); }
    for(i=0;i<4;i++) { fscanf(paramsFile, "%f ", &kl[i]); }
    for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &Tl[i]); }
    fclose(paramsFile);

  } else {
    printf("Unknown/unsupported format specified to BaseStereoCamera::loadCalibParameters\n");
    fclose(paramsFile);
    return 1;
  }
  createMaps();

  // Don't know rectified intrinsics and baseline from old params file. What should I set them to?
  paramsValid = true;
  return 0; // would only fail if file isn't accessible
}

void FrStereoRectMap::printCameraParams() {

}




/*******************************************************************************
 * Create, Load, Save, Set, Get calib map
 *
 */
void FrStereoRectMap::createMaps() {
  // Allocate or reallocate maps
  if(!rightXMap) rightXMap = new FrMatF(width, height, 1);
  if(!rightYMap) rightYMap = new FrMatF(width, height, 1);
  if(!leftXMap) leftXMap = new FrMatF(width, height, 1);
  if(!leftYMap) leftYMap = new FrMatF(width, height, 1);

  // FIXME: check map width and height and realloc if it doesn't match current width and height!
  //if(rightXMap->getWidth() != width || rightXMap->getHeight() != height) {
  //  printf("FrStereoRectMap: changing width and height not supported after initial map creation!\n");
  //  exit(1);
  //}

  // create the left and right maps
  createMap(rightXMap, rightYMap, Ar, kr, Tr);
  createMap(leftXMap, leftYMap, Al, kl, Tl);

  mapsValid = true;
}

// We now have rightXMap, rightYMap, leftXMap, leftYMap, all pointers to FrMatF
// How about rightA, rightK, rightT? Or stick with what I've got. Whatever.
// NOTE: maps must be pre-allocated!
void FrStereoRectMap::createMap(FrMatF *xMap, FrMatF *yMap, float *A, float *k, float *T) {

  //float* c = rect_coeff;
  float sx = 1.0/A[0];
  float sy = 1.0/A[4];
  float u0 = A[2];
  float v0 = A[5];
  float k1 = k[0];
  float k2 = k[1];
  float p1 = k[2];
  float p2 = k[3];
  //CvPoint2D32f* curMapPoint;
  float *cPtX, *cPtY;

  for(int v=0; v<height; v++) {
    cPtX = xMap->getDptr(v);
    cPtY = yMap->getDptr(v);
    for(int u=0; u<width; u++) {
      float xr = (T[0]*u + T[1]*v + T[2]) / (T[6]*u + T[7]*v + T[8]);
      float yr = (T[3]*u + T[4]*v + T[5]) / (T[6]*u + T[7]*v + T[8]);
      float du = xr-u0;
      float dv = yr-v0;
      float x = du*sx; // check for x=0; div by x below would result in div0 error!
      float y = dv*sy; // check for y=0; div by y below would result in div0 error!
      float r2 = x*x + y*y;
      float r4 = r2*r2;
      cPtX[u] = xr + (k1*r2*du) + (k2*r4*du) + (p2*r2*du/x) + (2*p2*x*du) + (2*p1*y*du);
      cPtY[u] = yr + (k1*r2*dv) + (k2*r4*dv) + (p1*r2*dv/y) + (2*p1*y*dv) + (2*p2*x*dv); 
      // FIXME: bounds check the results and clamp values to be inside the image (?)
      // ultimately, the coordinates -1,-1 will be used for anything outside the image and that will be 
      // replaced with black pixels in the bilinear filter.
      // Actually, the opencv function convertMap handles bounds checking acceptably for when I use cvUndistort.
      // I'll have to consider bounds when I make my own bilinear filter.
    }
  }
}

FrMatF* FrStereoRectMap::getMap(RectMapType mtype) {
  // FIXME: add cases for the 2 and 4 channel combined maps
  switch(mtype) {
  case right_x: return rightXMap;
  case right_y: return rightYMap;
  case left_x: return leftXMap;
  case left_y: return leftYMap;
  default: return 0;
  }
}


void FrStereoRectMap::setMap(FrMatF *newmap, RectMapType mtype) {}


/*
IplImage* FrStereoRectMap::readCalibMap(const char *xMapFile, const char *yMapFile) {
  FILE *xmap = fopen(xMapFile, "r");
  assert(xmap);
  FILE *ymap = fopen(yMapFile, "r");
  assert(ymap);
  IplImage *map = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 2);

  for(int y=0;y<height;y++) {
    float *dptr = (float*)(map->imageData + y*map->widthStep);
    for(int x=0;x<width;x++) {
      fscanf(xmap, "%f", dptr++);
      fscanf(ymap, "%f", dptr++);
    }
  }
  fclose(xmap);
  fclose(ymap);
  return map;
}

void FrStereoRectMap::readMaps(const char *rightXMapFile, const char *rightYMapFile, 
			     const char *leftXMapFile, const char *leftYMapFile) {
  rightMap = readCalibMap(rightXMapFile, rightYMapFile);
  leftMap = readCalibMap(leftXMapFile, leftYMapFile);
}

int FrStereoRectMap::loadCalibMaps(const char* mapfile) {
  int x, y;
  float xr, xl, yr, yl;
  int fmt;
  FILE *mapf = fopen(mapfile, "r");
  assert(mapf);

  // Check that this file has the expected format
  fscanf(mapf, "frcalibparamsfileversion %d", &fmt);
  if(fmt!=100) {
    printf("FrStereoRectMap::loadCalibMaps: unsupported file format in %s, unable to read file\n", mapfile);
    fclose(mapf);
    return 1;
  }

  rightMap = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 2);
  leftMap = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 2);
  // FIXME: reset maps to the ignore value once the hardware supports an ignore value
  while(fscanf(mapf, "%d %d %f %f %f %f", &x, &y, &xr, &yr, &xl, &yl)==6) {
    *((float*)(rightMap->imageData + y*rightMap->widthStep + (x*2)*sizeof(float))) = xr;
    *((float*)(rightMap->imageData + y*rightMap->widthStep + ((x*2)+1)*sizeof(float))) = yr;
    *((float*)(leftMap->imageData + y*leftMap->widthStep + (x*2)*sizeof(float))) = xl;
    *((float*)(leftMap->imageData + y*leftMap->widthStep + ((x*2)+1)*sizeof(float))) = yl;
  }
  fclose(mapf);
  return 0;
}

int FrStereoRectMap::saveCalibMaps(const char* mapfile) {
  return 0;
}
*/

/*******************************************************************************
 * Operator=
 */
void FrStereoRectMap::operator=(const FrStereoRectMap& src) {
  // Delete old maps if they exist...

  // Copy basic stuff
  width = src.width;
  height = src.height;
  fixTileSize();
  paramsValid = src.paramsValid;
  mapsValid = src.mapsValid;

  // Copy all params
  for(int i=0; i<9; i++) {
    Ar[i] = src.Ar[i];
    Al[i] = src.Al[i];
    Tr[i] = src.Tr[i];
    Tl[i] = src.Tl[i];
    An[i] = src.An[i];
  }
  for(int i=0; i<4; i++) {
    kr[i] = src.kr[i];
    kl[i] = src.kl[i];
  }
  B = src.B;

  // copy maps
  if(!paramsValid and mapsValid) {
    printf("FrStereoRectMap::operator= ERROR: Copying maps not implemented yet! Fixed rect map data won't be copied.\n");
    mapsValid = false;
    // FIXME: this is trivial if operator= is implemented for FrMatF!
  } else {
    createMaps();
  }
}


/// Correctly sets internal tile params based on current image width and height.
void FrStereoRectMap::fixTileSize() {
  // This calculation should be run anytime width and height are changed,
  // including when params files are loaded
  if(width%128 == 0) { widthTiles = width/128; } 
  else { widthTiles = (width/128) + 1; }
  widthSubTiles = widthTiles * 4;

  if(height%16 == 0) { heightTiles = height/16; } 
  else { heightTiles = (height/16) + 1; }
}

/// Returns length of Fr3RemapStruct int array (in ints).
int FrStereoRectMap::getFr3RemapStructLen() {
  return (widthTiles*128*heightTiles*16*2);
}

/*******************************************************************************
 * Create, Get, etc. Hardware formatted remap coordinates
 *
 */
int* FrStereoRectMap::getFr3RemapStruct() {
  assert(mapsValid);
  fixTileSize();
  // Maybe return a structure with length and data instead of just a simple array pointer.

  // malloc the hw_remap_coords array; width x height x 2images x 4 bytes/coord
  // array is initially shorts (16 bit) so that I can easily assign the 16 bit x and y
  // without any bit manipulation. It is recast to 32 bit ints at the end.
  if(hw_remap_coords) free(hw_remap_coords);
  unsigned short* map_coords = (unsigned short*)malloc(widthTiles*128*heightTiles*16*2*4);
  
  // Copy coordinates into the hw_remap_coords array in the correct format and order
  // Image is 6 tiles x 30 tiles or 768x480 for the default 752x480 case 
  // The calib block always iterates over subtiles of 32x16, switching sides every subtile
  // The calib block always operates on full tiles of 128x16.
  // The calib block uses 6 bits of sub-pixel precision, thus the multiply by 64;
  // all coordinates are interperted in hardware as 10.6 fixed point numbers
  // FIXME: set any off-image coordinates to the keyed no-lookup value to conserve cache resources
  // FIXME: Bounds checking:
  // I currently only check bounds on going over x in the result map
  // I need to also check bounds for going over y in the result map
  // I need to add bounds checking for all coords so that they don't go outside the src img.
  int map_entry = 0;
  int x, y, x_pixel, y_pixel;
  float *xpptr, *ypptr;
  for (int y_tile = 0; y_tile < heightTiles; y_tile++) {
    for (int x_tile = 0; x_tile < widthSubTiles; x_tile++) {

      // Right side
      for (y_pixel = 0; y_pixel < 16; y_pixel++) {
	for (x_pixel = 0; x_pixel < 32; x_pixel++) {
	  y = y_tile*16 + y_pixel;
	  //pix = (CvPoint2D32f*)(rightMap->imageData + y*rightMap->widthStep);
	  xpptr = rightXMap->getDptr(y);
	  ypptr = rightYMap->getDptr(y);
	  x = x_tile*32 + x_pixel;

	  if(x<width) {
	    map_coords[map_entry++] = (unsigned short)(xpptr[x] * 64);
	    map_coords[map_entry++] = (unsigned short)(ypptr[x] * 64);
	  } else {
	    map_coords[map_entry++] = (unsigned short)(xpptr[751] * 64);
	    map_coords[map_entry++] = (unsigned short)(ypptr[751] * 64);
	  }

	}
      }

      // Left side
      for (y_pixel = 0; y_pixel < 16; y_pixel++) {
	for (x_pixel = 0; x_pixel < 32; x_pixel++) {
	  y = y_tile*16 + y_pixel;
	  //pix = (CvPoint2D32f*)(leftMap->imageData + y*leftMap->widthStep);
	  xpptr = leftXMap->getDptr(y);
	  ypptr = leftYMap->getDptr(y);
	  x = x_tile*32 + x_pixel;

	  if(x<width) {
	    map_coords[map_entry++] = (unsigned short)(xpptr[x] * 64);
	    map_coords[map_entry++] = (unsigned short)(ypptr[x] * 64);
	  } else {
	    map_coords[map_entry++] = (unsigned short)(xpptr[751] * 64);
	    map_coords[map_entry++] = (unsigned short)(ypptr[751] * 64);
	  }

	}
      }

    }
  } 

  // Re-cast the array to int's to easily load it into the eagle device
  hw_remap_coords = (unsigned int*)map_coords;
  return (int*)hw_remap_coords;
}

////////////////////////////////////////////////////////////////////////////////
// Old and depricated functions
//
//
////////////////////////////////////////////////////////////////////////////////
/*
IplImage* FrStereoRectMap::getUndistMapFloat(float* A, float* dist_coeff) {
  IplImage *map = cvCreateImage(cvSize(752,480), IPL_DEPTH_32F, 2);
  float sx = 1.0/A[0];
  float sy = 1.0/A[4];
  float u0 = A[2];
  float v0 = A[5];
  float k1 = dist_coeff[0];
  float k2 = dist_coeff[1];
  float p1 = dist_coeff[2];
  //float p1 = 0;
  float p2 = dist_coeff[3];
  //float p2 = 0;
  CvPoint2D32f* curMapPoint;

  for(int v=0; v<480; v++) {
    curMapPoint = (CvPoint2D32f*)(map->imageData + v*map->widthStep);
    for(int u=0; u<752; u++) {
      float du = u-u0;
      float dv = v-v0;
      float x = du*sx;
      float y = dv*sy;
      float r2 = x*x + y*y;
      float r4 = r2*r2;
      curMapPoint[u].x = u + (k1*r2*du) + (k2*r4*du) + (p2*r2*du/x) + (2*p2*x*du) + (2*p1*y*du);
      curMapPoint[u].y = v + (k1*r2*dv) + (k2*r4*dv) + (p1*r2*dv/y) + (2*p1*y*dv) + (2*p2*x*dv); 
    }
  }
  return map;
}

IplImage* FrStereoRectMap::getRectMapFloat(float c[9]) {
  IplImage *map = cvCreateImage(cvSize(752,480), IPL_DEPTH_32F, 2);
  CvPoint2D32f* curMapPoint;

  for(int v=0; v<480; v++) {
    curMapPoint = (CvPoint2D32f*)(map->imageData + v*map->widthStep);
    for(int u=0; u<752; u++) {
      curMapPoint[u].x = (c[0]*u + c[1]*v + c[2]) / (c[6]*u + c[7]*v + c[8]);
      curMapPoint[u].y = (c[3]*u + c[4]*v + c[5]) / (c[6]*u + c[7]*v + c[8]);
    }
  }
  return map;
}
*/


