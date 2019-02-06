///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005 Focus Robotics. All rights reserved. 
//
// Proprietary and Confidential
//
// Creation_Date :  Tue Dec  6 2005
// Created by    :  Jason Peck
//
///////////////////////////////////////////////////////////////////////////
#include "cv.h"
#include "highgui.h"
#include "frcam.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include "fravi.h"

#define MAXVAL 255

int main(int argc, char**argv)
{
	char *in_filename, *out_filename;

	if (argc < 3) {
		printf("usage: %s in_filename out_filename\n",
		       argv[0]);
		exit(1);
	}
	in_filename = argv[1];
	out_filename = argv[2];

	printf("demo: opening input file %s\n", in_filename);
	fravi_input_file in_avi;
	in_avi.open_file(in_filename);

	printf("demo: opening output file %s\n", out_filename);
	fravi_file out_avi;
	out_avi.open_file(out_filename, 0, in_avi.get_frames_per_second());


	/* used for color image */
	IplImage *disp;
	int x =0;
	int y =0;
	int disparity = 0;
	int maxvalue = 0;

	if (MAXVAL < 12)
	  maxvalue = 7;
	else if (MAXVAL < 24) 
	  maxvalue = 15;
	else if (MAXVAL < 48)
	  maxvalue =31;
	else if (MAXVAL < 96)
	  maxvalue = 63;
	else if (MAXVAL < 192)
	  maxvalue = 127;
	else 
	  maxvalue = 255;

	// create new image (color image)
	disp =  cvCreateImage(cvSize(in_avi.get_width(), in_avi.get_height()), 
			      IPL_DEPTH_8U, 3);
	disp->origin = 1;
	cvSetZero(disp);

	IplImage* in_frame;

	cvNamedWindow("test", CV_WINDOW_AUTOSIZE);

	for(int i =0; i < 30*60; i++) {
		// read input image
		in_frame = in_avi.get_frame();

		if (NULL == in_frame)
			break;

		// create the colored image
		for ( y = 0; y < disp->height; y++ ) {

			for ( x = 0; x < (disp->width); x++ ) {

			  disparity = ((uchar*)(in_frame->imageData + in_frame->widthStep*y))[x];

			  if ( disparity == 0 ) {
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+2] = 0; // red
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+1] = 0; // green
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3] = 0; // blue
			  } else if ( disparity > (maxvalue/2) ) {
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+2] = 255; // red
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+1] = (255 - 2*(255-(disparity*(256/(maxvalue+1))))); // green
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3] = 0;   // blue	      
			  } else {
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+2] = (255- 2*(255-(disparity*256/(maxvalue+1)))); // red
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3+1] = 0; // green
			    ((uchar*)(disp->imageData + disp->widthStep*y))[x*3] =  (2*(255-(disparity*256/(maxvalue+1))));  // blue	      
			  }
			}
		}
		cvShowImage("test", disp);
		if (cvWaitKey(2) >= 0)
			break;

		out_avi.add_frame(disp);
	}
	
	out_avi.close_file();
}
