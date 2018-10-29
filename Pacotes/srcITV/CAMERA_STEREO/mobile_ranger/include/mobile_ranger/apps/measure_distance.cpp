/****************************************************************************
 * Copyright (c) 2009 by Focus Robotics. All rights reserved.
 *
 * This program is an unpublished work fully protected by the United States 
 * copyright laws and is considered a trade secret belonging to the copyright
 * holder. No part of this design may be reproduced stored in a retrieval 
 * system, or transmitted, in any form or by any means, electronic, 
 * mechanical, photocopying, recording, or otherwise, without prior written 
 * permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Thu Feb 26 2009
 * 
 * Brief Description: Simple example of looking at Focus Robotics camera output.
 * 
 * Functionality:
 * 
 * Issues:
 * Should display the point being measured on the images
 * Possibly should display the distance on the images
 * Can I smooth the disp map so that it's less likely to get a non-answer?
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/

#include <stdio.h>
#include "FrOptArgs.h"
#include "FrEagleStereoSmartCam.h"
#include "FrImageLUTProc.h"
#include "highgui.h"

CvPoint pt;

// define mouse callback for pixel highlighting
void on_mouse( int event, int x, int y, int flags, void* param ) {
  switch ( event ) {
  case CV_EVENT_LBUTTONDOWN:
    pt = cvPoint(x, y);
    break;
  }
}

int main(int argc, char *argv[]) {
  FrOptArgs *args = new FrOptArgs(argc, argv);
  FrEagleStereoSmartCam *essc = new FrEagleStereoSmartCam(args);
  FrImageLUTProc *lutProc = new FrImageLUTProc(LUT_TYPE_COLOR_SPECTRUM, 0, 0, 63);

  cvNamedWindow( "Disp", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Disp", 0, 20);
  cvNamedWindow( "Right", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Right", 780, 20);

  FrImage *disp, *right;
  essc->setChanType(0, ITYPE_FINAL_DISP);
  essc->setChanType(1, ITYPE_RIGHT_RECT);
  essc->grabImg();

  // Parameters for Distance measurement
  int   disparity;
  float focalLength = essc->getCameraFocalLength();
  float baseline = essc->getCameraBaseline();
  float numerator = focalLength * baseline * 3.28084;
  float distance;

  // set MouseCallBack
  cvSetMouseCallback("Disp", on_mouse,0);
  cvSetMouseCallback("Right", on_mouse,0);

  for(;;) {
    disp = essc->retrieveImg(0);
    right = essc->retrieveImg(1);
    essc->run();
    essc->grabImg();

    //if(fnum%20==0) {
    disparity = disp->getPixel(pt.x, pt.y);
    distance = numerator/disparity;
    printf("Distance to point is %f feet, disparity is %d\n", distance, disparity);
    //}

    lutProc->setImg(ITYPE_NONE, disp);
    lutProc->run();
    FrImage *remapped = lutProc->getImg(ITYPE_REMAPPED_LUT);
    cvShowImage( "Disp", remapped->getIplImage(0));
    cvShowImage( "Right", right->getIplImage(0));
    if ( cvWaitKey(2) >= 0 ) break;
  }
  return EXIT_SUCCESS;
}

