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
#include "FrOptArgs.h"
#include "FrEagleStereoSmartCam.h"
#include "FrImageLUTProc.h"
#include "highgui.h"

int main(int argc, char *argv[]) {
  FrOptArgs *args = new FrOptArgs(argc, argv);
  FrEagleStereoSmartCam *essc = new FrEagleStereoSmartCam(args);
  FrImageLUTProc *lutProc = new FrImageLUTProc(LUT_TYPE_COLOR_SPECTRUM, 0, 0, 63);

  cvNamedWindow( "Disp", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Disp", 0, 20);
  cvNamedWindow( "Right", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Right", 780, 20);

  FrImage *disp, *right;
  FrImage *blend = new FrImage(752, 480, 3); // FIXME: this should be created based on the size of disp&right, not fixed numbers

  essc->setChanType(0, ITYPE_FINAL_DISP);
  essc->setChanType(1, ITYPE_RIGHT_RECT);
  essc->grabImg();

  for(;;) {
    disp = essc->retrieveImg(0);
    right = essc->retrieveImg(1);
    essc->run();
    essc->grabImg();

    lutProc->setImg(ITYPE_NONE, disp);
    lutProc->run();
    FrImage *remapped = lutProc->getImg(ITYPE_REMAPPED_LUT);

    // Now create blend by alpha blending remapped with right
    // Should I create an alpha blending proc, or just do it here?
    float dwt = 0.25; // weight of color disparity in the blend
    float iwt = 0.80; // weight of right image in the blend
    for(int y=0; y<480; y++) {
      for(int x=0; x<752; x++) {
	for(int c=0; c<3; c++) {
	  float pix = (remapped->getPixel(x,y,c) * dwt) + (right->getPixel(x,y) * iwt);
	  uchar upix = (uchar)pix;
	  blend->setPixel(upix, x, y, c);
	}
      }
    }

    cvShowImage( "Disp", remapped->getIplImage(0));
    cvShowImage( "Right", blend->getIplImage(0));
    if ( cvWaitKey(2) >= 0 ) break;
  }
  return EXIT_SUCCESS;
}

