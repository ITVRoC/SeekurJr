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
  essc->setChanType(0, ITYPE_FINAL_DISP);
  essc->setChanType(1, ITYPE_RIGHT_RECT);
  essc->grabImg();

  int color = 1;
  for(;;) {
    disp = essc->retrieveImg(0);
    right = essc->retrieveImg(1);
    essc->run();
    essc->grabImg();

    lutProc->setImg(ITYPE_NONE, disp);
    lutProc->run();
    FrImage *remapped = lutProc->getImg(ITYPE_REMAPPED_LUT);
    cvShowImage( "Disp", remapped->getIplImage(0));
    cvShowImage( "Right", right->getIplImage(0));

    int keypressed = cvWaitKey(2) & 0xFF;
    if(keypressed == 'q') {
      break;
    } else if(keypressed=='c') { // toggle color mode for disp
      color = !color;
      if(color) lutProc->setLUTType(LUT_TYPE_COLOR_SPECTRUM);
      else lutProc->setLUTType(LUT_TYPE_FULL_RANGE);
    }
  }
  return EXIT_SUCCESS;
}

