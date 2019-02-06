/****************************************************************************
 * Copyright (c) 2008 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Sat Oct 25 2008
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
#include "FrImage.h"
#include "FrImageLUTProc.h"
#include "FrOptArgs.h"
#include "FrEagleStereoSmartCam.h"
#include "FrStereoSmartCamGUI.h"
#include "highgui.h"
#ifndef DEVSYS_MODE
#define DEVSYS_MODE 0
#endif
#define PROC_ENABLES 3

// This function will be registered with OpenCV highgui so that a point on the
// window may be clicked and it's distance reported. The data pointer will point
// to a 2 int structure of x,y which is the current point.
CvPoint pt;

// define mouse callback for pixel highlighting
void on_mouse( int event, int x, int y, int flags, void *data) {
  switch ( event ) {
  case CV_EVENT_LBUTTONDOWN:
    pt = cvPoint(x, y);
    break;
  }
}

void draw_point_distance(FrImage img, int x, int y, int disp, float fl, float b) {

}

int main(int argc, char *argv[]) {
  FrOptArgs *args = new FrOptArgs(argc, argv);
  FrEagleStereoSmartCam *essc = new FrEagleStereoSmartCam(args);
  essc->setDevsysMode(DEVSYS_MODE);
  essc->setProcEnables(PROC_ENABLES);

  cvNamedWindow( "Disp", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Disp", 0, 20);
  cvNamedWindow( "Right", CV_WINDOW_AUTOSIZE );
  cvMoveWindow("Right", 780, 20);
  // FIXME: add a control and status window like in opencv_simple:
  // Maybe make my own FrStereoSmartCamStatusTool class, similar to the frStatusTool Jason wrote
  // The status tool would be passed the essc and a window name.
  // There would be a method on the status tool to make the window visible or not.
  FrStereoSmartCamGUI *gui = new FrStereoSmartCamGUI(essc, "Control Window");

  FrImage *disp, *right;
  FrImageLUTProc *lutProc = new FrImageLUTProc(LUT_TYPE_COLOR_SPECTRUM, 0, 0, 63);

  // state variables determine the current mode
  int show_disp = 1;
  int color_disp = 1;
  int show_distance = 0;
  int show_statuswin = 1;
  //int recording = 0;
  int adjustment_mode = 0; // 0=match qual, 1=desired bin, 2=exposure, 3=gain, 4=framerate
  int current_mqc = 64; // range 0-127
  int current_bin = 35; // range 1-64;
  int current_exposure = 240; // range 1-480
  int current_gain = 32; // range 16-64
  int current_fps = 30;
  int update_disp = !DEVSYS_MODE;
  int update_right = !DEVSYS_MODE;

  //essc->setProcMode(FR_SSC_PIPELINED_STREAMING); // this is the default
  essc->setChanType(0, ITYPE_FINAL_DISP);
  essc->setChanType(1, ITYPE_RIGHT_RECT);
#if PROC_ENABLES==0
  show_disp = 0;
  essc->setChanType(0, ITYPE_LEFT_RAW);
  essc->setChanType(1, ITYPE_RIGHT_RAW);
#endif
  essc->setAegcMode(3);
  essc->setAegcSkip(3);
  essc->setCamBrightness(35);
  essc->grabImg(); // specify channel or type, default is all channels currently set up (ITYPE_ALL or chan -1)

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

    if(show_distance) {
      disparity = disp->getPixel(pt.x, pt.y);
      distance = numerator/disparity;
      printf("Distance to point is %f feet, disparity is %d\n", distance, disparity);
    }

    if(update_disp) {
      if(color_disp && show_disp) {
	lutProc->setImg(ITYPE_NONE, disp);
	lutProc->run();
	FrImage *remapped = lutProc->getImg(ITYPE_REMAPPED_LUT);
	cvShowImage("Disp", remapped->getIplImage(0));
      } else {
	cvShowImage("Disp", disp->getIplImage(0));
      }
    }
    if(update_right) {
      cvShowImage("Right", right->getIplImage(0));
    }

    gui->update();
    // User Interface
    // FIXME: Add hotkeys to:
    // X colorize disp or not
    // show distance point or not
    // X show disp or left rect in disp window
    // X show raw or rectified images (r&l depending on last setting)
    // change framerate
    // scale images (1/2 size, etc.)
    // X stop updating images
    // start recording a video file
    // snap image current images into pgm or pnm files
    // X adjust brightness, gain, exposure, mqc, search range (can also set with stat tool)
    // make control and status window appear or dissappear
    // change verbosity of messages
    // dump status and register values
    int keypressed = cvWaitKey(2) & 0xFF;
    if(keypressed == 'q') {
      break;
    } else if(keypressed=='l') { // display left img in disp window
      essc->setChanType(0, ITYPE_LEFT_RECT);
      show_disp = 0;
    } else if(keypressed=='d') { // display disp img in disp window
      essc->setChanType(0, ITYPE_FINAL_DISP);
      show_disp = 1;
    } else if(keypressed=='w') { // display raw images
      if(!show_disp) essc->setChanType(0, ITYPE_LEFT_RAW);
      essc->setChanType(1, ITYPE_RIGHT_RAW);
    } else if(keypressed=='t') { // display rect images
      if(!show_disp) essc->setChanType(0, ITYPE_LEFT_RECT);
      essc->setChanType(1, ITYPE_RIGHT_RECT);
    } else if(keypressed=='0') { // update neither image (reduced cpu usage)
      update_disp = update_right = 0;
    } else if(keypressed=='1') { // update only disp image
      update_disp = 1; update_right = 0;
    } else if(keypressed=='2') { // update only right image
      update_disp = 0; update_right = 1;
    } else if(keypressed=='3') { // update both images
      update_disp = update_right = 1;
    } else if(keypressed=='m') { // set match qual (use arrow keys to adjust when selected?)
      printf("Adjustment mode set to Match Quality Adjustment. Use arrow keys to adjust\n");
      adjustment_mode = 0;
    } else if(keypressed=='b') { // set desired bin and enable AEGC (also use arrow keys to set brightness?)
      printf("Adjustment mode set to Desired Bin Adjustment. Use arrow keys to adjust\n");
      adjustment_mode = 1;
    } else if(keypressed=='e') { // set desired exposure and disable AEGC (also used arrow keys)
      printf("Adjustment mode set to Manual Exposure Adjustment. Use arrow keys to adjust\n");
      adjustment_mode = 2;
    } else if(keypressed=='g') { // set desired gain and disable AEGC (adj with arrow keys)
      printf("Adjustment mode set to Manual Gain Adjustment. Use arrow keys to adjust\n");
      adjustment_mode = 3;
    } else if(keypressed=='f') { // set/show framerate (use arrows to adjust)
      printf("Adjustment mode set to Frame Rate Adjustment. Use arrow keys to adjust\n");
      adjustment_mode = 4;
    } else if(keypressed=='c') { // toggle colorize disp
      color_disp = !color_disp;
    } else if(keypressed=='z') { // toggle show distance to selected point
      show_distance = !show_distance;
    } else if(keypressed=='s') { // toggle show status and control window
      show_statuswin = !show_statuswin;
      if(!show_statuswin) gui->hide();
      else gui->show();
    } else if(keypressed==' ') { // start/stop recording movie or snap picture

    } else if(keypressed==190) { // F1: restart without reloading firmware
      // shut down camera by deleting essc (currently leaks memory due to broken destructors)
      delete(essc);
      // recreate essc, set params, grab first frame
      essc = new FrEagleStereoSmartCam(args);
      gui->setStereoSmartCam(essc);
      essc->setDevsysMode(DEVSYS_MODE);
      essc->setProcEnables(PROC_ENABLES);
      essc->setChanType(0, ITYPE_FINAL_DISP);
      essc->setChanType(1, ITYPE_RIGHT_RECT);
#if PROC_ENABLES==0
      show_disp = 0;
      essc->setChanType(0, ITYPE_LEFT_RAW);
      essc->setChanType(1, ITYPE_RIGHT_RAW);
#endif
      essc->grabImg(); // specify channel or type, default is all channels currently set up (ITYPE_ALL or chan -1)

    } else if(keypressed==191) { // F2: reload firmware and restart
      // shut down camera by deleting essc (currently leaks memory due to broken destructors)
      delete(essc);
      // reload firmware with external program
      if(int rc=system("./jtag_reload")) {
	printf("jtag_reload returned error code %d\n", rc);
      }
      // wait some time to allow the FPGA to reload
      usleep(2000000);
      // recreate essc, set params, grab first frame
      essc = new FrEagleStereoSmartCam(args);
      gui->setStereoSmartCam(essc);
      essc->setDevsysMode(DEVSYS_MODE);
      essc->setProcEnables(PROC_ENABLES);
      essc->setChanType(0, ITYPE_FINAL_DISP);
      essc->setChanType(1, ITYPE_RIGHT_RECT);
#if PROC_ENABLES==0
      show_disp = 0;
      essc->setChanType(0, ITYPE_LEFT_RAW);
      essc->setChanType(1, ITYPE_RIGHT_RAW);
#endif
      essc->grabImg(); // specify channel or type, default is all channels currently set up (ITYPE_ALL or chan -1)

    } else if(keypressed==192) { // F3: save left and right rectified images to image files
      // This won't work if we aren't currently capturing and displaying r+l rect unless
      // I change it to capture one frame of those, save, and then switch back.
      // FIXME: maybe save disp image as well?

      // This time code is to create automatic filenames for the images
      time_t curtime = time(NULL);
      struct tm *ctm = localtime(&curtime);
      printf("%4.4d%2.2d%2.2d_%2.2d%2.2d%2.2d\n", ctm->tm_year+1900, ctm->tm_mon+1, ctm->tm_mday, ctm->tm_hour, ctm->tm_min, ctm->tm_sec);

      // change channels, grab, retrieve, save, change back
      disp = essc->retrieveImg(0); // only doing these retrieve's to make sure current DMA's are done
      right = essc->retrieveImg(1);
      essc->run();
      essc->setChanType(0, ITYPE_LEFT_RECT);
      essc->setChanType(1, ITYPE_RIGHT_RECT);
      essc->grabImg();

      disp = essc->retrieveImg(0); // These retrieve's are guarenteed to get the images I want
      right = essc->retrieveImg(1);
      // FIXME: save the image pair here!
      essc->run();
      essc->setChanType(0, ITYPE_LEFT_RECT); // FIXME: change to what they were before!
      essc->setChanType(1, ITYPE_RIGHT_RECT);
      essc->grabImg(); // This just sets things back up so the loop can continue from where it left off
      
    } else if(keypressed==193) { // F4: compute frame checksum (detect hang support)

    } else if(keypressed==194) { // F5: save current disp and right img, whatever mode

    } else if(keypressed==195) { // F6: Set Manual AEGC
      essc->setAegcMode(0);
    } else if(keypressed==196) { // F7: Set image sensor AEGC
      essc->setAegcMode(1);
    } else if(keypressed==197) { // F8: Set FPGA AEGC
      essc->setAegcMode(2);
    } else if(keypressed==198) { // F9: Set Soft AEGC
      essc->setAegcMode(3);
    } else if(keypressed==199) { // F10: Set Ind AEGC
      essc->setAegcMode(4);
    } else if(keypressed==200) { // F11: 
    } else if(keypressed==201) { // F12: 

    } else if(keypressed==82) { // up arrow: adjust settings based on mode
      switch(adjustment_mode) {
      case 0: { // match qual
	if(++current_mqc > 127) current_mqc = 127;
	essc->setMatchQualThresh(current_mqc);
	printf("Set Match Quality Threshold to %d\n", current_mqc);
	break;
      }
      case 1: { // desired bin
	if(++current_bin > 64) current_bin = 64;
	essc->setCamBrightness(current_bin);
	printf("Set Desired Bin to %d\n", current_bin);
	break;
      }
      case 2: { // exposure
	if(++current_exposure > 480) current_exposure = 480;
	essc->setCamExposure(current_exposure);
	printf("Set Manual Exposure to %d\n", current_exposure);
	break;
      }
      case 3: { // gain
	if(++current_gain > 64) current_gain = 64;
	essc->setCamGain(current_gain);
	printf("Set Manual Gain to %d\n", current_gain);
	break;
      }
      case 4: { // frame rate
	if(current_fps<=55) {
	  current_fps+=5;
	  essc->setFrameRate(current_fps);
	  printf("Set frame rate to %d\n", current_fps);
	}
	break;
      }
      }
    } else if(keypressed==84) { // down arrow: adjust settings based on mode
      switch(adjustment_mode) {
      case 0: { // match qual
	if(--current_mqc < 0) current_mqc = 0;
	essc->setMatchQualThresh(current_mqc);
	printf("Set Match Quality Threshold to %d\n", current_mqc);
	break;
      }
      case 1: { // desired bin
	if(--current_bin < 1) current_bin = 1;
	essc->setCamBrightness(current_bin);
	printf("Set Desired Bin to %d\n", current_bin);
	break;
      }
      case 2: { // exposure
	if(--current_exposure < 1) current_exposure = 1;
	essc->setCamExposure(current_exposure);
	printf("Set Manual Exposure to %d\n", current_exposure);
	break;
      }
      case 3: { // gain
	if(--current_gain < 16) current_gain = 16;
	essc->setCamGain(current_gain);
	printf("Set Manual Gain to %d\n", current_gain);
	break;
      }
      case 4: { // frame rate
	if(current_fps>=10) {
	  current_fps-=5;
	  essc->setFrameRate(current_fps);
	  printf("Set frame rate to %d\n", current_fps);
	}
	break;
      }
      }
    }
    //} else if(keypressed<255) printf("%d %x\n", keypressed, keypressed);
    // 82 up arrow, 84 down arrow
    // 83 right arrow, 81 left arrow
    // 190 F1, 194 F5, 198 F9
    // 10 CR 225 left shift, 226 right shift, 8 backspace, 9 tab, 27 esc
  }

  // Clean up
  delete(essc);
  delete(args);
  return EXIT_SUCCESS;
}



