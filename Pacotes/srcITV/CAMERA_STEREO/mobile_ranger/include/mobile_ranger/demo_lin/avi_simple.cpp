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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>


#include "frcam.h"
#include "fravi.h"

sig_atomic_t stop = 0;

void SIGINT_handler(int sig)
{
	signal(sig, SIG_IGN);
	stop = 1;
}

int main(int argc, char **argv)
{
	FrChannel *channel[4];
	IplImage *frame[4];
	int type;
	int nbuffer = 0;
	int frame_cnt = 0;
	int i;

	fravi_file fr_avifile[4];
	char *filename[4];
	const char *prefix;
	float fps = 0.0f;

	struct sigaction sa;

	if ((argc < 3) || (argc > 7)) {
    		printf("usage: %s output_prefix input_type ... \n"
		       "API example program to output raw video file with fravi.\n"
		       "Up to 4 inputs can be captured simultaniously.\n"
		       "The output format will play on linux with mplayer, and can be\n"
		       "compressed using mencoder for playback on windows:\n"
		       "mencoder input.avi -ovc lavc -lavcopts vcodec=msmpeg4v2 -o output.avi"
		       "\n", argv[0]);
		exit(1);
	}
	prefix = argv[1];

	printf("demo: prefix=%s\n", prefix);

  
	/* FIXME: estimate fps */
	nbuffer = argc - 2;
	fps = 15;

	for(i = 0; i < nbuffer; i++) {
		/* get free channel */
	  channel[i] = frOpenChannel(i,0);
		if (!channel[i]) {
			printf("demo: frOpen failed for device%d\n",i);
			exit(1);			
		}

		/* configure the channel to the specified input type */
		printf("demo: converting %s to int\n", argv[i+2]);
		type = atoi(argv[i+2]);
		printf("demo: trying to set channel[%d] type %d\n", i, type);	       
		frSetChannelInput(channel[i], type);
		printf("demo: channel[%d] set to type %d\n",i, type);	       
		filename[i] = (char*)malloc(sizeof(prefix)+50);
		sprintf(filename[i], "%s_%d.avi", prefix, type);
		printf("demo: opening avi file %s", filename[i]);	       
		fr_avifile[i].open_file(filename[i], 1, (int)fps);
	}

	/* program calibration remap coords */
	FrCalibMap *map = frOpenCalibMap("/etc/fr/default.calib");
	if(frProgRemapData(channel[0], map)) {
		fprintf(stderr, "RemapCoords failed\n");
		exit(1);
	}
	frCloseCalibMap(map);
  
	/* register a signal handler */
	sigemptyset (&sa.sa_mask);
	sa.sa_flags = 0;
	sa.sa_handler = SIGINT_handler;
	sigaction (SIGINT, &sa, 0);

	printf("Press CTRL+C to stop capturing.\n");

	/* used to calculate fps */
	struct timeval startTv, thisTv;
	double startTime, lastTime, thisTime;
	gettimeofday(&startTv, 0);
	startTime = startTv.tv_sec + (startTv.tv_usec/1000000.0);
	lastTime = startTime;

	for(frame_cnt = 1; !stop ;frame_cnt++)	{
		for (i = 0; i < nbuffer; i++) {
			if (stop)
				goto out;

			if(frGrabFrame(channel[i]))
				goto out;
		}

		for (i = 0; i < nbuffer; i++) {
			if (stop)
				goto out;


			frame[i] = frRetrieveFrame(channel[i]);
			if (NULL == frame) 
				goto out;
		}

		for (i = 0; i < nbuffer; i++) {
			if (stop)
				goto out;
			
			//frame[i]->origin = 1;

			fr_avifile[i].add_frame(frame[i]);
		}

		if(frame_cnt%100==0) {
		  gettimeofday(&thisTv, 0);
		  thisTime = thisTv.tv_sec + (thisTv.tv_usec/1000000.0);
		  fps = 100/(thisTime-lastTime);
		  printf("%f fps over last 100 frames, %f fps culumative\n", 
			 fps, 
			 frame_cnt/(thisTime-startTime));
		  lastTime = thisTime;
		}
	}

out:
	/* return ndepth video channels */
	for(i = 0; i < nbuffer; i++) {
		frCloseChannel(channel[i]);
		fr_avifile[i].close_file();
	}

	printf("demo: %d frames saved\n", frame_cnt);
}



