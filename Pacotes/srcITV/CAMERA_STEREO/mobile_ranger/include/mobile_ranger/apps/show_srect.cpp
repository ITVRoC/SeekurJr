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
 * Brief Description: Simplest example of looking at Focus Robotics camera output.
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
#include "highgui.h"

int main(int argc, char *argv[]) {
  FrOptArgs *args = new FrOptArgs(argc, argv);
  FrEagleStereoSmartCam *essc = new FrEagleStereoSmartCam(args);

  cvNamedWindow( "Left", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Left", 0, 20);
  cvNamedWindow( "Right", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Right", 780, 20);

  FrImage *left, *right;
  essc->setChanType(0, ITYPE_LEFT_RECT);
  essc->setChanType(1, ITYPE_RIGHT_RECT);
  essc->grabImg();

  for(;;) {
    left = essc->retrieveImg(0);
    right = essc->retrieveImg(1);
    essc->run();
    essc->grabImg();
    cvShowImage( "Left", left->getIplImage(0));
    cvShowImage( "Right", right->getIplImage(0));
    if ( cvWaitKey(2) >= 0 ) break;
  }
  return EXIT_SUCCESS;
}

