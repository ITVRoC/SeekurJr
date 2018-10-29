///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2006 Focus Robotics. All rights reserved. 
//
// Proprietary and Confidential
//
// Created by    :  Jason Peck
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the 
// Free Software Foundation; either version 2 of the License, or (at your 
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but 
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
// General Public License for more details.
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


int main( int argc, char** argv )
{
	FrChannel *channel;
	FrVcam    *vcam[2];
	IplImage  *in_frame[2];
	IplImage  *out_frame;
	int	  i;
	
	/* check arguments */
	if ((argc < 3)) {
    		printf("usage: %s right_src_image left_src_image disp_out_image \n"
		       "API example program to load calibrated grayscale images into hardware.\n"
		       "\n", argv[0]);
		exit(1);
	}

	/* open the input image */
	for(i = 0; i < 2; i++) {
		in_frame[i] = cvLoadImage(argv[1+i], 0); // 0 forces grayscale
		if (NULL == in_frame[i]) {
			printf("demo: cvLoadImage failed for %s\n", argv[1+i]);
			exit(1);
		}
	}

	/* get unconfigured ndepth video channel, device supports 4 [0-3] */
	channel = frOpenChannel(0,0);
	if (!channel) {
		printf("demo: frOpenChannel failed for device 0");
		exit(1);
	}

	/* get unconfigured ndepth virtual camera, device supports 2 [4-5] */
	for(i = 0; i < 2; i++) {
		vcam[i] = frOpenVcam(4+i);
		if (!vcam[i]) {
			printf("demo: frOpenVcam failed for device%d\n",4+i);
			exit(1);			
		}
	}

	/* configure the output vcams as calibrated left and right */
	if (frSetVcamOutput(vcam[0], FR_VCAM_OUTPUT_TYPE_CAL_RT)) {
		printf("demo: set OUTPUT_TYPE failed for device 4\n");
		exit(1);
	}
	if (frSetVcamOutput(vcam[1], FR_VCAM_OUTPUT_TYPE_CAL_LT)) {
		printf("demo: set OUTPUT_TYPE failed for device 5\n");
		exit(1);
	}

	/* set output format for vcams to match input image */
	if (frSetVcamFmt(vcam[0], in_frame[0])) {
		printf("demo: frSetVcamFmt failed for device 4\n");
		exit(1);
	}
	if (frSetVcamFmt(vcam[1], in_frame[1])) {
		printf("demo: frSetVcamFmt failed for device 5\n");
		exit(1);
	}

	/* configure the input channel as disparity.
	   image format will automatically match vcam */
	if(frSetChannelInput(channel, FR_CHAN_INPUT_TYPE_DISP)) {
		printf("demo: set INPUT_TYPE failed for device 0\n");
		exit(1);
	}

	/* queue an incomming buffer */
	if (frGrabFrame(channel)) {
		printf("demo: unable to queue an incomming buffer\n");
		exit(1);
	}
 			
	/* write the output image to hardware */
	for (i = 0; i < 2; i++ ) {
		if (frWriteVcamFrame(vcam[i], in_frame[i])) {
			printf("demo: unable to write image to hardware\n");
			exit(1);
		}
	}

	/* step the hardware process to advance the pipeline  */
	if (frStepVcam(channel)) {
		printf("demo: unable to step hardware process\n");
		exit(1);
	}

	/* dequeue incomming buffer, discard it since it is one back in pipeline */
	frRetrieveFrame(channel);

	/* queue an incomming buffer */
	if (frGrabFrame(channel)) {
		printf("demo: unable to queue an incomming buffer\n");
		exit(1);
	}

	/* step the hardware process again to fill incomming buffer */
	if (frStepVcam(channel)) {
		printf("demo: unable to step hardware process\n");
		exit(1);
	}

	/* dequeue incomming buffer into an IplImage */
	out_frame = frRetrieveFrame(channel);
	if (NULL == out_frame) {
		printf("demo: incomming buffer was null\n");
	}
 
	/* write image out to disk */
#if CV_MAJOR_VERSION < 2
	cvSaveImage(argv[3], out_frame);
#else
	cvSaveImage(argv[3], out_frame, 0);
#endif

	/* finish up cleanly */
	frCloseChannel(channel);
	frCloseVcam(vcam[0]);
	frCloseVcam(vcam[1]);
}

