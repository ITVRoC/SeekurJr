///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2006-2009 Focus Robotics. All rights reserved. 
//
// Created by    :  Andrew Worcester
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
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <sys/time.h>
#include <ctype.h>

const int etalonWidth = 6; // For the 6x4 chessboard
const int etalonHeight = 4; // For the 6x4 chessboard
const int etalonNumPoints = etalonWidth * etalonHeight;
const int frameWidth = 752;
const int frameHeight = 480;

double fr_stereo_rectcheck(IplImage *srcLeft, IplImage *srcRight, IplImage *dstLeft, IplImage *dstRight, int verbose=0) {
        CvPoint2D32f* cornersLeft = (CvPoint2D32f *)malloc((etalonNumPoints + 1) * sizeof(CvPoint2D32f));	
	CvPoint2D32f* cornersRight = (CvPoint2D32f *)malloc((etalonNumPoints + 1) * sizeof(CvPoint2D32f));	
	int ccountLeft, ccountRight;
	int retvall, retvalr;
	float total_offset = 0.0;

	// Try to find the corners for the chess board in the last image grabbed
	retvall = cvFindChessboardCorners(srcLeft, cvSize(etalonWidth,etalonHeight), cornersLeft, &ccountLeft, 0);
	retvalr = cvFindChessboardCorners(srcRight, cvSize(etalonWidth,etalonHeight), cornersRight, &ccountRight, 0);

	// Draw a circle at each corner found
	if(dstLeft != NULL && dstRight != NULL) {
                cvCvtColor(srcLeft, dstLeft, CV_GRAY2BGR);
		cvCvtColor(srcRight, dstRight, CV_GRAY2BGR);
		cvDrawChessboardCorners(dstLeft, cvSize(etalonWidth,etalonHeight), cornersLeft, ccountLeft, retvall);
		cvDrawChessboardCorners(dstRight, cvSize(etalonWidth,etalonHeight), cornersRight, ccountRight, retvalr);
	}

	// Calculate offsets at all etalon corners if all the corners were found
	if(retvall > 0 && retvalr > 0) { 
                cvFindCornerSubPix(srcLeft, cornersLeft, ccountLeft, cvSize(5,5), cvSize(-1, -1), 
				   cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1));
		cvFindCornerSubPix(srcRight, cornersRight, ccountRight, cvSize(5,5), cvSize(-1, -1), 
				   cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1));
    
		for(int i=0; i<etalonNumPoints; i++) {
		  float offset = fabsf(cornersRight[i].y - cornersLeft[i].y);
		  total_offset += offset;
		  if(verbose)
		    std::cout << "Point " << i << " right y val " << cornersRight[i].y << " left y val " 
			      << cornersLeft[i].y << " offset " << offset << "\n";
		}
		if(verbose)
		        std::cout << "Total offset for image is " << total_offset << "\n";
	} else {
	        total_offset = -1.0;
		if(verbose)
		        std::cout << "Frame grab failed! Not all chess board corners could be found.\n";
	}

	free(cornersLeft);
	free(cornersRight);
	return total_offset;
}


int main( int argc, char** argv )
{
	FrChannel *channel[3];
	IplImage *frame[3];
	char wndname[3][50];

	int i;

	/* get unconfigured ndepth video channels, device supports up to 4 */
	for(i = 0; i < 2; i++) {
	  channel[i] = frOpenChannel(i,0);
		if (!channel[i]) {
			printf("demo: frOpen failed for device%d\n",i);
			exit(1);			
		}
	}
	

	/* configure the channels as disparity and rt calibrated */
	if(frSetChannelInput(channel[0], FR_CHAN_INPUT_TYPE_CAL_LT)) {
		printf("demo: set INPUT_TYPE failed for device 0\n");
		exit(1);
	}
	if(frSetChannelInput(channel[1], FR_CHAN_INPUT_TYPE_CAL_RT)) {
		printf("demo: set INPUT_TYPE failed for device 1\n");
		exit(1);
	}
	

	/* program calibration remap coords */
	FrCalibMap *map = frOpenCalibMap("/etc/fr/default.calib");
	if(frProgRemapData(channel[0], map)) {
		fprintf(stderr, "RemapCoords failed\n");
		exit(1);
	}
	frCloseCalibMap(map);


	/* create display windows for each channel */
	sprintf(wndname[0], "Left");	
	sprintf(wndname[1], "Right");
	cvNamedWindow(wndname[0], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[0], 0, 0);
	cvNamedWindow(wndname[1], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[1], 0, 520);


	/* add status/control interface */
	int frameCount = 0;

	FrStatusTool *stat = frOpenStatusTool(channel[0], "Status");
	
	frAddBinMon(stat);
	frAddExposureMon(stat);
	frAddGainMon(stat);
	frAddAEGCMon(stat);
	frAddFPSMon(stat, &frameCount);

	frAddDesiredBinTbar(stat);
	frAddManualExpTbar(stat);
	frAddManualGainTbar(stat);
	
	frAddI2CKeypress(stat);

	int keypressed;
	double offset;
	int verbose = 0;

	// create images to display corners found on the checkerboard
	CvSize captSize = cvSize(frameWidth,frameHeight);
	IplImage* cornerLeft = cvCreateImage(captSize, IPL_DEPTH_8U, 3);
	IplImage* cornerRight = cvCreateImage(captSize, IPL_DEPTH_8U, 3);


	// frGrabFrame will start the DMA for each channel 
	for (i = 0; i < 2; i++) {
		if(frGrabFrame(channel[i]))
			goto out;
	}		

	/* get and display the images */
	for(;;)	{	

		// frRetreiveFrame will map the DMA buffer to userspace 
		for (i = 0; i < 2; i++) {

			frame[i] = frRetrieveFrame(channel[i]);
			if (NULL == frame) 
				goto out;
		}

		// calculate the checkerboard corners and rectification offset
		offset = fr_stereo_rectcheck(frame[1], frame[0], cornerLeft, cornerRight, verbose);
		if(offset>=0) {
		        offset = offset / etalonNumPoints;
		        std::cout << "Average pixel offset is " << offset << "\n";
		}


		// display the frames
		cvShowImage(wndname[0], cornerRight);
		cvShowImage(wndname[1], cornerLeft);


		// frGrabFrame will start the DMA for each channel 
		for (i = 0; i < 2; i++) {
			if(frGrabFrame(channel[i]))
				goto out;
		}		

		frameCount++;

		// update monitors every 100 frames dma'd
		if(frameCount%10==0) {
			frUpdateMonitors(stat);
		}

		// check pressed key, if any
		if( -1 == frCheckKeypress(stat, keypressed = cvWaitKey(2)))
			break;
		else if(keypressed == 32) // option spacebar to toggle verbosity
		        verbose = !verbose;
	}


out:
	frCloseStatusTool(stat);

	/* return ndepth video channels */
	for(i = 0; i < 2; i++) {
		frCloseChannel(channel[i]);
	}

}

