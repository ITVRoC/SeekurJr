/*///////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev.h>

#include <pgm.h>

#include "frcam.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

/// a buffer to hold frames
struct buffer {
        void *          start;
        size_t          length;
	int		height;
	int		width;
	int		bytesperline;
};

/* Global Variables */
static char *           dev_name        = NULL;
static int              fd              = -1;
struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;
static gray		**pgm_data;
static int		frame_count	= 0;
static char		pgm_name[100];

/**************************************************/
static void errno_exit (const char *s) {
	fprintf (stderr, "%s error %d, %s\n",
		 s, errno, strerror (errno));

	exit (EXIT_FAILURE);
}
/**************************************************/

/**************************************************/
static int xioctl (int fd,
                   int request,
                   void *arg) {
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

/*   printf("doing IOCTL\n"); */

	return r;
}
/**************************************************/

/**************************************************/
static void open_device (void) {
	struct stat st; 

	if (-1 == stat (dev_name, &st)) {
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",
			 dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	if (!S_ISCHR (st.st_mode)) {
		fprintf (stderr, "%s is no device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	fd = open (dev_name, O_RDWR /* required */, 0);

	if (-1 == fd) {
		fprintf (stderr, "Cannot open '%s': %d, %s\n",
			 dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
}
/**************************************************/

/**************************************************/
static void init_device() {

	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_requestbuffers req;
	FrCalibMap *map;
	int index;

	/** query capabilities -- for sanity check **/
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is not a V4L2 device\n", dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
  
	if ( !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ) {
		fprintf(stderr, "%s is not a capture device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	if ( !(cap.capabilities & V4L2_CAP_STREAMING) ) {
		fprintf(stderr, "%s does not support streaming\n", dev_name);
	}

	/* set the input type to be disparity */
	index = FR_CHAN_INPUT_TYPE_DISP;
	if (-1 == ioctl(fd, VIDIOC_S_INPUT, &index)) {
		fprintf(stderr, "FR ERROR:%s VIDIOC_S_INPUT : %d, %s\n",
			dev_name, errno, strerror (errno));
		return;
	}


	/* get format */
	CLEAR (fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
		errno_exit("VIDIOC_G_FMT");
	}

	printf("%s height=%d, width=%d, bytesperline=%d\n",
	       dev_name, fmt.fmt.pix.height, fmt.fmt.pix.width, fmt.fmt.pix.bytesperline);

	/* try format */
	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
		printf("Couldn't set here\n");
		fflush(stdout);
		errno_exit("VIDIOC_S_FMT");
	}

	/** initialize mem map **/
	CLEAR (req);

	req.count  = 2;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support mem map\n", 
				dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 1) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", 
			dev_name);
		exit (EXIT_FAILURE);
	}

	buffers = calloc (req.count, sizeof (struct buffer));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;

		if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf)) {
			errno_exit("VIDIOC_QUERYBUF");
		}

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap ( NULL,
			       buf.length,
			       PROT_READ | PROT_WRITE,
			       MAP_SHARED,
			       fd,
			       buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start) {
			errno_exit("mmap");
		}

		/* save format as well */

		buffers[n_buffers].height = fmt.fmt.pix.height;
		buffers[n_buffers].width = fmt.fmt.pix.width;
		buffers[n_buffers].bytesperline = fmt.fmt.pix.bytesperline;
	}

	/* program calibration remap coords */
	map = frOpenCalibMap("/etc/fr/default.calib");
	if(-1 == ioctl(fd, FR3_IOC_PROGRAM_REMAP_COORDS, frGetRemapData(map))) {
	  fprintf(stderr, "FR ERROR: Unable to program remap data : %d, %s\n",
		  errno, strerror (errno));
	  exit(1);
	}

	//if(-1 == frProgramRemapCoords(map, fd)) {
	//	fprintf(stderr, "RemapCoords failed\n");
	//	exit(1);
	//}
	frCloseCalibMap(map);

	/* setup pgm file to write images to */
	pgm_data = pgm_allocarray(fmt.fmt.pix.width, fmt.fmt.pix.height);


}
/**************************************************/

/**************************************************/
void start_capturing() {
	unsigned int i;
	enum v4l2_buf_type type;

               
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
		errno_exit ("VIDIOC_STREAMON");


}
/**************************************************/


/**************************************************/
void process_image(struct buffer *frame) {
	int y, x;
	int width = frame->width;
	int height = frame->height;
	int bytesperline = frame->bytesperline;        

	sprintf ( pgm_name ,  "%d_v4l2_simple.pgm", frame_count);	
	frame_count++;

	printf("writing %s height=%d, width=%d, bytesperline=%d\n",
	       pgm_name, height, width, bytesperline);

	for ( y = 0; y < height; y++ ) {
		int yStep = y*bytesperline;
		for (  x = 0; x < width; x++ ) {
			pgm_data[y][x] =  ((char*)(frame->start + yStep))[x];
		}
	}

	FILE* pgm_file = pm_openw(pgm_name);
	pgm_writepgm(pgm_file, pgm_data, width, height, 255, 0);
	pm_close(pgm_file);
}
/**************************************************/

/**************************************************/
void mainloop() {
	struct v4l2_buffer buf;
	int i, j;

	for (j = 0; j < 5; j++) {

		for (i = 0; i < n_buffers; ++i) {
			CLEAR(buf); 
		
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
			buf.memory = V4L2_MEMORY_MMAP; 
			buf.index = i;
		
			if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
				errno_exit ("VIDIOC_QBUF");
		
		}     
		
		for (i = 0; i < n_buffers; ++i) {
			CLEAR(buf); 
		
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
			buf.memory = V4L2_MEMORY_MMAP; 
		
			if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) { 
				switch (errno) { 
				case EAGAIN: 
					printf("DQBUF FAILED\n");
					return; 
				default: 
					errno_exit ("VIDIOC_DQBUF"); 
				} 
			}

			process_image (&buffers[buf.index]);
		}
	}

}
/**************************************************/

/**************************************************/
void stop_capturing() {
	enum v4l2_buf_type type;
	
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE; 
	
	if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
		errno_exit ("VIDIOC_STREAMOFF");
}
/**************************************************/

/**************************************************/
void uninit_device() {
	unsigned int i;

	for (i = 0; i < n_buffers; ++i) 
		if (-1 == munmap (buffers[i].start, buffers[i].length))
			errno_exit("munmap");

	free (buffers);
}
/**************************************************/

/**************************************************/
void close_device() {
	if (-1 == close (fd))
		errno_exit ("close");

	fd = -1;
}
/**************************************************/

/**************************************************/
int main (int argc, char **argv) {
	dev_name = "/dev/video0";

	/**  Open device **/
	printf("Opening device\n");
	fflush(stdout);
	open_device();

	/** Initialize Device **/
	printf("Initialize device\n");
	fflush(stdout);
	init_device();

	/** Start Capturing **/
	printf("start capturing\n");
	fflush(stdout);
	start_capturing();

	/** main loop **/
	printf("main loop\n");
	fflush(stdout);
	mainloop();

	/** stop capturing **/
	printf("stop capturing\n");
	fflush(stdout);
	stop_capturing();

	/** uninitialize device **/
	printf("unitialize device\n");
	fflush(stdout);
	uninit_device();

	/** close device **/
	printf("close device\n");
	fflush(stdout);
	close_device();

	exit (EXIT_SUCCESS);

	return 0;
  
}
/**************************************************/
