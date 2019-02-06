///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005 Focus Robotics. All rights reserved. 
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
#include "cxcore.h" /* IplImage and its allocation routines */
#include "frcam.h"


void frCreateColorLUT(int maxvalue, 
		      uchar *redLut, uchar *greenLut, uchar *blueLut) {
	int disparity;

	for(disparity=0; disparity<maxvalue; disparity++) {
		if ( disparity == 0 ) {
			redLut[disparity] = 0; // red
			greenLut[disparity] = 0; // green
			blueLut[disparity] = 0; // blue
		} else if ( disparity > (maxvalue/2) ) {
			redLut[disparity] = 255; // red
			greenLut[disparity] = (255 - 2*(255-(disparity*(256/(maxvalue+1))))); // green
			blueLut[disparity] = 0;   // blue	      
		} else {
			redLut[disparity] = (255- 2*(255-(disparity*256/(maxvalue+1)))); // red
			greenLut[disparity] = 0; // green
			blueLut[disparity] =  (2*(255-(disparity*256/(maxvalue+1))));  // blue	      
		}
		//printf("disp=%d, red=%d green=%d blue=%d\n", disparity, redLut[disparity], greenLut[disparity], blueLut[disparity]);
	}
}


void frRemapImageColor(IplImage *disp, IplImage *color, 
		       char *redLut, char *greenLut, char *blueLut) {
	char *dispLine = disp->imageData;
	char *colorLine = color->imageData;
	unsigned char *dispPix;
	char *colorPix;
	int y, x;

	for(y=0; y<disp->height; y++) {
		dispLine += disp->widthStep;
		colorLine += color->widthStep;
		dispPix = (unsigned char*)dispLine;
		colorPix = colorLine;
		for(x=0; x<disp->width; x++) {
			*colorPix++ = blueLut[*dispPix];
			*colorPix++ = greenLut[*dispPix];
			*colorPix++ = redLut[*dispPix++];
		}
	}
}
