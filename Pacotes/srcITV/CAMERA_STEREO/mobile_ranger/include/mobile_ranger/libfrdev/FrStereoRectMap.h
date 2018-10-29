/****************************************************************************
 * Copyright (c) 2006-2008 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Fri Dec 19 2006
 * 
 * Brief Description:
 * 
 * Functionality:
 * 
 * Issues:
 * The name of this class doesn't seem just right. It contains all stereo camera
 * parameters, but it's more then that. It also can create rectification maps,
 * or even just read maps without camera parameters. It supports rectification,
 * but doesn't actually do rectification, that's the rectifier class. This class
 * could also produce a transform which doesn't rectify at all. It doesn't calc
 * the transform to rectify, just creates the maps from the distortion params
 * and rect transform matrices.
 *
 * Could call it:
 * FrCalibMap -- that's what's used in the sample programs
 * FrRectMap -- maybe more accurate
 * EagleCalibMap -- what it used to be called, but I'm prepending everything with Fr now
 * FrStereoCameraParameters -- OK, but ignores map generation and is long
 * FrStereoCameraParams -- shorter is better, still ignores maps
 * FrStereoRectMap <--- I'll use this one because it is a rect map for both images in a stereo pair
 * FrStereoCameraRectMap
 *
 * I may actually want to have another class which holds more camera param
 * information like the fundamental or essential matrix, or arrays or corresponding
 * points, or parameters for different camera models. Maybe that is untimately the
 * StereoCameraParameters and this RectMap class holds one of those--or not...
 * It's weird because the rect map needs some of the stereo camera params, but
 * not all of them.
 *
 * How do I program the maps in hardware consistantly for different programming
 * interfaces--like between eagle and fr3 drivers? Probably by returning the map
 * and letting the higher layer program it instead of actually programming it
 * in this code. But how does that work? Always create a new map and pass the 
 * RectMap class to the StereoCamera class, via the initCalib method. Inside the 
 * initCalib method, the code retrieves the pointer to the int array and copies 
 * that into the hardware, either via writeMEM or ioctl. Maybe initCalib tries to
 * do something smart if it's passed a null pointer instead of a RectMap, like 
 * making it's own RectMap and using default calib file locations, or from the
 * environment or command line.
 *
 * The FrStereoRectProc class actually performs the rectification transfer, or
 * works as an interface for hardware that performs the transfer. Eventually, 
 * the RectMap and RectProc classes could merge into the RectProc. This would
 * also mirror intended hardware changes where the hardware starts generating 
 * maps it's self.
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/

/* Started as: EagleCalibMap.h
 * This class started out as just converting params to a hw map and loading that
 * map into hardware. It seems to be transforming into all stereo params so 
 * perhaps it should be renamed to StereoCameraParameters.
 * In addition to holding stereo camera parameters, it also creates the map used
 * to convert raw images into rectified images, which seems like the more 
 * important function. 
 * Currently, it holds intrinsics and distortion for each camera in the stereo
 * pair as well as the transform to rectify the cameras and the effective
 * intrinsics of the rectified stereo camera, and it's baseline.
 */
#ifndef FRSTEREORECTMAP_H
#define FRSTEREORECTMAP_H

#include "FrImage.h"
#include "FrMatF.h"

enum RectMapType {right_x, right_y, left_x, left_y, right_xy, left_xy, full};

/// Encapsulates stereo camera params and read and writes them in various formats.
class FrStereoRectMap {
 public:
  // ***************************************************************************
  // Constructors and Destructors

  // Default constructor should take no arguments and automagically find the 
  // calib file... but it needs to know the image sensor id number in order to
  // do that. Does it need to know anything else? Currently I find the 
  // appropriate calib file using the sensor id number and some env variables
  // in the eagleStereoSmartCam class, but I would like to push that to this
  // class. I just need a way to pass in a unique identifier, I think.
  //FrStereoRectMap(int uid);

  // Map defaults to identity mapping with no distortion
  // FIXME: probably don't need these args to have defaults.
  // loadCalibParameters and readCalibMap should reset width and height based
  // on values in those files
  FrStereoRectMap(int width, int height);
  // It might be useful to have a constructor which just gets the filename for 
  // a calib or map file. How does it determine which type it has?
  // Probably add optional type parameter to this constructor
  FrStereoRectMap(const char* filename);
  // Copy Constructor
  // What should be copied over? Filename? Params? Coord LUT?
  FrStereoRectMap(const FrStereoRectMap &map);
  // Initialization/assignment Constructor
  //FrStereoRectMap& operator=(const FrStereoRectMap &rhs);

  ~FrStereoRectMap();


  // ***************************************************************************
  // *** Set and Get and Load and Save camera parameters and transforms ***
  // Some values are camera parameters, some are rectification parameters; should names reflect that?
  // *** Set functions
  void setIdentityParams();
  void setCameraParams(float *A0, float *k0, float *T0, 
		       float *A1, float *k1, float *T1, 
		       float *Anew, float Base);
  // allow setting individual params, need enum for side and params, also allow setting from FrMatF
  void setCameraParam(int side, int param, float value); // generic version not yet implemented
  void setCameraParam(int side, int param, FrMatF *value); // generic version not yet implemented

  // *** Get functions
  float getCameraParam(int side, int param); // generic version not yet implemented
  float getCameraBaseline() {return B;}
  float getCameraFocalLength() {return An[0];}
  FrMatF* getCameraParamIntrnsMat(int cam);
  FrMatF* getCameraParamDistCoeff(int cam);
  FrMatF* getCameraParamTrnsfmMat(int cam);

  // *** Load, save, and print
  // Format options for load and save: fmt=-1 means autoselect for load
  // fmt=1 is the new FR calib params file, fmt=0 is the old opencv calib params file, other values reserved
  int loadCameraParams(const char* filename, int fmt=-1); // may fail if file doesn't exist or isn't readable
  int saveCameraParams(const char* filename, int fmt=1); // may fail if file isn't writable or params aren't set
  // just for debug
  void printCameraParams();

  // ***************************************************************************
  // *** Creating, Loading, Saving Maps ***
  // Convert camera parameters into a map
  // It's an error if calib parameters aren't set when this is called
  // maybe this should be protected as well and just called automatically when params are changed
  void createMaps();

 protected:
  // This is the implementation of createMaps
  void createMap(FrMatF *xMap, FrMatF *yMap, float *A, float *k, float *T);

 public:
  // New functions to read/write maps in different formats. May be called up to
  // four times for formats that store map data in multiple files. Each part of 
  // the data and separate file gets it's own fmt number.
  // Need to define formats; keep map and param file formats in one namespace so a 
  // constructor can take one fmt arg to read any type of file.
  // FIXME: anything that directly sets the map should set params invalid
  int readMapFile(const char *mapFile, int fmt);
  int writeMapFile(const char *mapFile, int fmt);

  // setMap and getMap would round out the set
  // Data is always stored as 4 separate map matrices, but may be passed in as
  // any of the mtype types.
  // Define types: right_x, right_y, left_x, left_y, right_xy, left_xy, full(4 chan mat)
  FrMatF* getMap(RectMapType mtype);
  void setMap(FrMatF *newmap, RectMapType mtype);

  // ***************************************************************************
  // *** Misc
  // make operator= copy all params and maps over
  void operator=(const FrStereoRectMap& src);

  // ***************************************************************************
  // *** Dealing with HW format maps ***
  // Return the map for a particular hardware platform. Different calls will 
  // support different required hardware formats as they are created.
  int* getFr3RemapStruct();
  int getFr3RemapStructLen();

  // ***************************************************************************
 protected:
  void fixTileSize();

  // ***************************************************************************
 private:
  int width;
  int height;
  // Tiles stuff is only used to format to Fr3 hardware usable map
  int widthTiles; // width in 128 pixel wide tiles--may be larger then actual image
  int widthSubTiles; // 4*widthTiles
  int heightTiles; // height in 16 pixel high tiles--may be larger then actual image

  // Parameters for each raw camera in the stereo pair
  // NOTE: 0 is right, 1 is left
  // CONSIDER: should I convert internal structures to FrMatf?
  float Ar[9];
  float Al[9];
  float kr[4]; //dist_coeff0;
  float kl[4]; //dist_coeff1;
  float Tr[9]; //rect_coeff0; Rect transform, don't confuse with T translation vec
  float Tl[9]; //rect_coeff1;
  // Add parameters for the stereo camera, and each rectified camera in the pair
  float An[9]; // New intrinsics, maybe Arect?
  float B;

 public:
  // These maps are single channel float maps, the size of the image. 
  FrMatF *rightXMap;
  FrMatF *rightYMap;
  FrMatF *leftXMap;
  FrMatF *leftYMap;

  // This map is data formatted to load directly into fr3 memory
  unsigned int* hw_remap_coords;

  bool paramsValid;
  bool mapsValid;
};

#endif
