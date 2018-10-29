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

#include <stdio.h>  // for FILE
#include <assert.h> // for assert
#include <errno.h>
#include <sys/ioctl.h>

#include "cxcore.h"
#include "frcam.h"

typedef struct FrCameraParam
{	
/*	intrinsics: [fx 0 cx; 0 fy cy; 0 0 1] */
	float   A[9];	
/*	distortion: k1, k2, p1, p2 */
	float	dist_coeff[4];
/*	rect transform matrix: 3x3 projective transform coefficients */
	float	rect_coeff[9];
} FrCameraParam;

/**
 * Map structure.
 *
 * one per device 
 */
struct FrCalibMap
{
	const char	*filename;
/*	image width in pixels */	
	int		width;	
	int		widthTiles;
	int		widthSubTiles;
/*	image height in pixels */
	int		height;
	int		heightTiles;
/*	intrinsics, distorition, transform for each camera */
	FrCameraParam   camera[2]; // 0 rt, 1 lt
/*	new rect intrinsics:  [fx 0 cx; 0 fy cy; 0 0 1] */
	float		An[10];
/*      distance in meters between the camera optical centers */
        float           baseline;

/*      These are generated after file load */
	unsigned int	*hw_remap_coords;
};

static int loadCalibParameters(FrCalibMap *map) 
{
        int i;

	printf("Opening %s\n", map->filename);
	FILE* paramsFile = fopen(map->filename, "r" );
	if (NULL == paramsFile) {
		fprintf(stderr, "FR ERROR: could not open calib map %s\n", 
			map->filename);
		return -1;
	}

	// Read first line into a string to determine file type
	// Call sub-funtion to load old-style opencv params files
	char line[256];
	int fmt = -1;
	fgets(line, 255, paramsFile);
	if(line[0]=='2') fmt=0;
	if(line[0]=='f') fmt=1; 

	/* Old-style format0 calib paramters file */
	if(fmt==0) {
	  float params[88];
	  for(i=0;i<88;i++) {
	    fscanf(paramsFile, "%f", &params[i]); /* FIXME: check return code and fail if no more params read */
	  }
    
	  /* Now put all that data into the FrCameraParams structure */
	  map->width  = (int)params[0];
	  map->height = (int)params[1];
	  
	  for(i=0;i<9;i++) {
		  map->camera[0].A[i]          = params[2+i];
		  map->camera[1].A[i]	       = params[29+i];
		  map->camera[0].rect_coeff[i] = params[70+i];
		  map->camera[1].rect_coeff[i] = params[79+i];
	  }
	  for(i=0;i<4;i++) {
		  map->camera[0].dist_coeff[i] = params[11+i];
		  map->camera[1].dist_coeff[i] = params[38+i];
	  }

	  /* baseline and new intrinsics are approximate since they aren't stored
	     in the old file format */
	  map->baseline = 0.06;
	  for(i=0;i<9;i++) { map->An[i] = params[2+i]; }

	/* New-style format1 calib parameters file */
	} else if(fmt==1) {
	  printf("NEW.1\n");
		fscanf(paramsFile, "%d %d %f\n", &map->width, &map->height, &map->baseline);
		for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &map->An[i]); }

		// rt
		for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &map->camera[0].A[i]); }
		for(i=0;i<4;i++) { fscanf(paramsFile, "%f ", &map->camera[0].dist_coeff[i]); }
		for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &map->camera[0].rect_coeff[i]); }

		// lt
		for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &map->camera[1].A[i]); }
		for(i=0;i<4;i++) { fscanf(paramsFile, "%f ", &map->camera[1].dist_coeff[i]); }
		for(i=0;i<9;i++) { fscanf(paramsFile, "%f ", &map->camera[1].rect_coeff[i]); }

	} else {
		fprintf(stderr, "FR ERROR: Unknown/unsupported calib map format specified %s\n", 
			map->filename);
		return -1;  
	}
	/* finished with param file */
	fclose(paramsFile);

	/* calculate tile offsets */
	if(map->width%128 == 0) {
		map->widthTiles = map->width/128;
	} else {
		map->widthTiles = (map->width/128) + 1;
	}
	map->widthSubTiles = map->widthTiles * 4;
	
	if(map->height%16 == 0) {
		map->heightTiles = map->height/16;
	} else {
		map->heightTiles = (map->height/16) + 1;
	}

	return 0;
}

/*
static void frPrintCameraParam(FrCameraParam *camera) {
	int j;

	for( j = 0; j < (int)(sizeof(FrCameraParam)/sizeof(float)); j++ ) {
		printf("%15.10f ", ((float*)(camera))[j]);
	}
	printf("\n\n");

}
*/

FrCalibMap* frOpenCalibMap(const char *filename) 
{
	FrCalibMap *map;

	map = (FrCalibMap*)malloc(sizeof(struct FrCalibMap));
	map->filename = filename;

	/* read the file */
	if(loadCalibParameters(map)) {
		fprintf(stderr, 
			"FR ERROR: loadCalibParameters failed, check filename\n");
		return NULL;
	}

	return map;
}

float frGetCameraBaseline(FrCalibMap *map) {
  return map->baseline;
}

float frGetCameraFocalLength(FrCalibMap *map) {
  return map->An[0];
}


IplImage* frCombMapFloat(float* A, 
			 float* dist_coeff, 
			 float* rect_coeff,
			 int	width,
			 int    height) 
{
	printf("\n");
	printf("Image intrinsics matrix is:\n%f %f %f\n%f %f %f\n%f %f %f\n", 
	       A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]);
	printf("Image distortion coeffs are:\n%f %f %f %f\n", 
	       dist_coeff[0], dist_coeff[1], dist_coeff[2], dist_coeff[3]);
	printf("Image rectification matrix is:\n%f %f %f\n%f %f %f\n%f %f %f\n", 
	       rect_coeff[0], rect_coeff[1], rect_coeff[2], rect_coeff[3], 
	       rect_coeff[4], rect_coeff[5], rect_coeff[6], rect_coeff[7], 
	       rect_coeff[8]);
	printf("Image width x height is: %d x %d\n", width, height);
	

	IplImage *map = cvCreateImage(cvSize(width,height), IPL_DEPTH_32F, 2);
	float* c = rect_coeff;
	float sx = 1.0/A[0];
	float sy = 1.0/A[4];
	float u0 = A[2];
	float v0 = A[5];
	float k1 = dist_coeff[0];
	float k2 = dist_coeff[1];
	float p1 = dist_coeff[2];
	float p2 = dist_coeff[3];
	CvPoint2D32f* curMapPoint;

	for(int v=0; v<height; v++) {
		curMapPoint = (CvPoint2D32f*)(map->imageData + v*map->widthStep); // base of map data plus v*widthstep?
		for(int u=0; u<width; u++) {
			float xr = (c[0]*u + c[1]*v + c[2]) / (c[6]*u + c[7]*v + c[8]);
			float yr = (c[3]*u + c[4]*v + c[5]) / (c[6]*u + c[7]*v + c[8]);
			float du = xr-u0;
			float dv = yr-v0;
			float x = du*sx; // check for x=0; div by x below would result in div0 error!
			float y = dv*sy; // check for y=0; div by y below would result in div0 error!
			float r2 = x*x + y*y;
			float r4 = r2*r2;
			curMapPoint[u].x = xr + (k1*r2*du) + (k2*r4*du) + (p2*r2*du/x) + (2*p2*x*du) + (2*p1*y*du);
			curMapPoint[u].y = yr + (k1*r2*dv) + (k2*r4*dv) + (p1*r2*dv/y) + (2*p1*y*dv) + (2*p2*x*dv); 
			// FIXME: bounds check the results and clamp values to be inside the image (?)
			// ultimately, the coordinates -1,-1 will be used for anything outside the image and that will be 
			// replaced with black pixels in the bilinear filter.
			// Actually, the opencv function convertMap handles bounds checking acceptably for when I use cvUndistort.
			// I'll have to consider bounds when I make my own bilinear filter.
		}
	}
	return map;
}


int frMakeHWRemapCoords(FrCalibMap *map) 
{  
	int width = map->width;
	int widthTiles = map->widthTiles;
	int widthSubTiles = map->widthSubTiles;
	int height = map->height;
	int heightTiles = map->heightTiles;

	/* create combined maps */
	IplImage *rightMap = frCombMapFloat(map->camera[0].A,
					    map->camera[0].dist_coeff,
					    map->camera[0].rect_coeff,
					    width, height);
	IplImage *leftMap  = frCombMapFloat(map->camera[1].A,
					    map->camera[1].dist_coeff,
					    map->camera[1].rect_coeff,
					    width, height);	

	// malloc the hw_remap_coords array; width x height x 2images x 4 bytes/coord
	// array is initially shorts (16 bit) so that I can easily assign the 16 bit x and y
	// without any bit manipulation. It is recast to 32 bit ints at the end.
	unsigned short* map_coords = (unsigned short*)malloc(widthTiles*128*heightTiles*16*2*4);

	// Copy coordinates into the hw_remap_coords array in the correct format and order
	// Image is 6 tiles x 30 tiles or 768x480 for the default 752x480 case 
	// The calib block always iterates over subtiles of 32x16, switching sides every subtile
	// The calib block always operates on full tiles of 128x16.
	// The calib block uses 6 bits of sub-pixel precision, thus the multiply by 64;
	// all coordinates are interperted in hardware as 10.6 fixed point numbers
	// FIXME: set any off-image coordinates to the keyed no-lookup value to conserve cache resources
	// FIXME: Bounds checking:
	// I currently only check bounds on going over x in the result map
	// I need to also check bounds for going over y in the result map
	// I need to add bounds checking for all coords so that they don't go outside the src img.
	int map_entry = 0;
	int x, y, x_pixel, y_pixel;
	CvPoint2D32f *pix;
	for (int y_tile = 0; y_tile < heightTiles; y_tile++) {
		for (int x_tile = 0; x_tile < widthSubTiles; x_tile++) {

			// Right side
			for (y_pixel = 0; y_pixel < 16; y_pixel++) {
				for (x_pixel = 0; x_pixel < 32; x_pixel++) {
					y = y_tile*16 + y_pixel;
					pix = (CvPoint2D32f*)(rightMap->imageData + y*rightMap->widthStep);
					x = x_tile*32 + x_pixel;

					if(x<width) {
						map_coords[map_entry++] = (unsigned short)(pix[x].x * 64);
						map_coords[map_entry++] = (unsigned short)(pix[x].y * 64);
					} else {
						map_coords[map_entry++] = (unsigned short)(pix[751].x * 64);
						map_coords[map_entry++] = (unsigned short)(pix[751].y * 64);
					}

				}
			}

			// Left side
			for (y_pixel = 0; y_pixel < 16; y_pixel++) {
				for (x_pixel = 0; x_pixel < 32; x_pixel++) {
					y = y_tile*16 + y_pixel;
					pix = (CvPoint2D32f*)(leftMap->imageData + y*leftMap->widthStep);
					x = x_tile*32 + x_pixel;

					if(x<width) {
						map_coords[map_entry++] = (unsigned short)(pix[x].x * 64);
						map_coords[map_entry++] = (unsigned short)(pix[x].y * 64);
					} else {
						map_coords[map_entry++] = (unsigned short)(pix[751].x * 64);
						map_coords[map_entry++] = (unsigned short)(pix[751].y * 64);
					}

				}
			}

		}
	} 

	// Re-cast the array to int's to easily load it into the device
	map->hw_remap_coords = (unsigned int*)map_coords;

	return 0;
}

int frProgramRemapCoords(FrCalibMap *map, int fd) 
{
	if(frMakeHWRemapCoords(map)) {
		fprintf(stderr, 
			"FR ERROR: frProgramRemapCoords failed, check file contents\n");
		return -1;
	}

	int chip_version = ioctl(fd, FR3_IOC_CHIP_VERSION, NULL);

	if(-1 == chip_version) {
		fprintf(stderr, "FR ERROR: couldn't get chip version");
		return -1;
	} else {
		printf("chip_version=%d\n", chip_version);
	}

	//int i;
	//printf("****** BEGIN REMAP COORDS *******\n");
	//for (i = 0; i < map->widthTiles*128*map->heightTiles*16*2; i++ ) {
	//	printf("%d\n", map->hw_remap_coords[i]);
	//}
	//printf("****** END   REMAP COORDS *******\n");

	if(-1 == ioctl(fd, FR3_IOC_PROGRAM_REMAP_COORDS, map->hw_remap_coords)) {
		fprintf(stderr, "FR ERROR: %s FR3_IOC_PROGRAM_REMAP_COORDS : %d, %s\n",
			map->filename, errno, strerror (errno));
		return -1;
	}
	return 0;
}

int* frGetRemapData(FrCalibMap *map) {
  if(frMakeHWRemapCoords(map)) {
    fprintf(stderr, "FR ERROR: frProgramRemapCoords failed, check file contents\n");
    return 0;
  }
  return (int*)map->hw_remap_coords;
}

int frGetRemapDataLen(FrCalibMap *map) {
  return map->widthTiles * 128 * map->heightTiles * 16 * 2;
}

void frCloseCalibMap(FrCalibMap* map) 
{
	free(map->hw_remap_coords);	
	free(map);
}
