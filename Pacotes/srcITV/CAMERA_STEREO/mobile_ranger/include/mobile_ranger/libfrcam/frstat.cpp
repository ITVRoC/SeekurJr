///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005-2009 Focus Robotics. All rights reserved. 
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

#include "cxcore.h"  /* IplImage and its allocation routines */
#include "highgui.h" /* trackbar, mouse, and other events */
#include <sys/time.h> /* gettimeofday */
#include "frcam.h"

#define DESIRED_BIN_TBARNAME "Desired Image Bin      "
#define MANUAL_EXP_TBARNAME  "Manual Exposure Value  "
#define MANUAL_GAIN_TBARNAME "Manual Gain Value      "
#define MATCH_QUAL_TBARNAME  "Match Quality Threshold"
#define FRSTAT_N_MONITORS 20

enum frstat_mon_type {
	FRSTAT_MON_TYPE_I2C      = 0,
	FRSTAT_MON_TYPE_IOCTL    = 1,
	FRSTAT_MON_TYPE_CALLBACK = 2,
	FRSTAT_MON_TYPE_PROPERTY = 3,
	FRSTAT_MON_TYPE_CAM_PROP = 4
};

struct FrRegField {
	int		offset;
	const char		*name;
	const char		*fmt;
	int		start;
	int		size;
};


struct FrMonitor {
	frstat_mon_type type;
	FrRegField	field;
	FrMonCallback   callback;
	void*		callback_param;
};

struct FrStatusTool {
/*	device channel */
	FrChannel	*channel;
/*	device handle */
	int		fd;

	const char		*wndname;
	IplImage	*frame;

	CvFont		dfont;
	CvScalar	font_color;
	
/*      Monitors */
	int		n_monitors;
	FrMonitor	monitors[FRSTAT_N_MONITORS];

/*	Trackbars */
	int		desired_bin_tbarvalue;
	int		last_desired_bin_tbarvalue;
	int		manual_exp_tbarvalue;
	int		last_manual_exp_tbarvalue;
	int		manual_gain_tbarvalue;
	int		last_manual_gain_tbarvalue;
        int             match_qual_tbarvalue;
        int             last_match_qual_tbarvalue;

/*	Key handlers */
	int		i2c_notify;
};

struct FrFPSData {
	struct timeval startTv;
	struct timeval thisTv;
	double startTime;
	double lastTime;
	double thisTime;
	int    *frameCount;
	int    lastCount;
	float  fps;
};

///////////////////////////////////////////////////////////////////////////
/// INITIALIZATION
///////////////////////////////////////////////////////////////////////////
FrStatusTool* frOpenStatusTool(FrChannel* channel, const char* name) {
  FrStatusTool *stat;
  
  stat = (FrStatusTool *)malloc(sizeof(FrStatusTool));

  if (NULL == stat) {
    fprintf(stderr, "FR ERROR: FrStatusTool malloc failed\n");
    return NULL;
  }
  memset(stat, 0, sizeof(FrStatusTool));
  stat->channel = channel;
  stat->fd = frGetChannelDev(channel);
  stat->wndname = name;
  stat->frame = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 3);
  
  cvNamedWindow(stat->wndname, CV_WINDOW_AUTOSIZE);
  cvShowImage(stat->wndname, stat->frame);

  float hscale = 0.5f;
  float vscale = 0.5f;
  float italicscale = 0.0f;
  int thickness = 1; 
  int line_type = 8;
  stat->font_color = CV_RGB(255, 0, 0);
  cvInitFont (&stat->dfont, CV_FONT_VECTOR0, 
	      hscale, vscale, italicscale, thickness, line_type);

  stat->desired_bin_tbarvalue = 0;
  stat->last_desired_bin_tbarvalue = -1;
  stat->manual_exp_tbarvalue = 0;
  stat->last_manual_exp_tbarvalue = -1;
  stat->manual_gain_tbarvalue = 0;
  stat->last_manual_gain_tbarvalue = -1;
  stat->match_qual_tbarvalue = 64;
  stat->last_match_qual_tbarvalue = -1;

  return stat;
}

///////////////////////////////////////////////////////////////////////////
/// MONITORS
///////////////////////////////////////////////////////////////////////////

static char* fps_monitor(FrStatusTool* stat, void* data) {
  FrFPSData *fps_data = (FrFPSData*) data;
  char* text = (char *)malloc(1024);

  gettimeofday(&fps_data->thisTv, 0);
  fps_data->thisTime = fps_data->thisTv.tv_sec + (fps_data->thisTv.tv_usec/1000000.0);
  fps_data->fps = (*fps_data->frameCount - fps_data->lastCount)/
    (fps_data->thisTime-fps_data->lastTime);

  //fps_data->fps = 100/(fps_data->thisTime-fps_data->lastTime);
  /*
    printf("%f fps over last 100 frames, %f fps culumative\n", 
    fps, 
    frameCount/(thisTime-startTime));
  */
  fps_data->lastTime = fps_data->thisTime;
  fps_data->lastCount = *fps_data->frameCount;
  
  sprintf(text, "FPS               %f", fps_data->fps);
  return text;
}

int frAddCallbackMonitor(FrStatusTool* stat, FrMonCallback callback, void* param) {
  FrMonitor *monitor = &stat->monitors[stat->n_monitors];
  monitor->type	        = FRSTAT_MON_TYPE_CALLBACK;
  monitor->callback       = callback;
  monitor->callback_param = param;
  stat->n_monitors++;
  return 0;
}

int frAddI2CMon(FrStatusTool* stat, int offset, 
	      const char* field_name, const char* field_fmt, 
	      int field_start, int field_size) {
  FrMonitor *monitor = &stat->monitors[stat->n_monitors];
  monitor->type = FRSTAT_MON_TYPE_I2C;
  monitor->field.offset = offset;
  monitor->field.name  = field_name;
  monitor->field.fmt   = field_fmt;
  monitor->field.start = field_start;
  monitor->field.size  = field_size;
  stat->n_monitors++;
  return 0;
}

int frAddIOCTLMon(FrStatusTool* stat, int offset, 
		  const char* field_name, const char* field_fmt, 
		  int field_start, int field_size) {
  FrMonitor *monitor = &stat->monitors[stat->n_monitors];
  monitor->type = FRSTAT_MON_TYPE_IOCTL;
  monitor->field.offset = offset;
  monitor->field.name  = field_name;
  monitor->field.fmt   = field_fmt;
  monitor->field.start = field_start;
  monitor->field.size  = field_size;
  stat->n_monitors++;
  return 0;
}

int frAddPropertyMon(FrStatusTool* stat, int offset, const char* field_name, const char* field_fmt) {
  FrMonitor *monitor = &stat->monitors[stat->n_monitors];
  monitor->type = FRSTAT_MON_TYPE_PROPERTY;
  monitor->field.offset = offset;
  monitor->field.name = field_name;
  monitor->field.fmt = field_fmt;
  stat->n_monitors++;
  return 0;
}

int frAddCamPropMon(FrStatusTool* stat, int offset, char* field_name, char* field_fmt) {
  FrMonitor *monitor = &stat->monitors[stat->n_monitors];
  monitor->type = FRSTAT_MON_TYPE_CAM_PROP;
  monitor->field.offset = offset;
  monitor->field.name = field_name;
  monitor->field.fmt = field_fmt;
  stat->n_monitors++;
  return 0;
}

int frAddExposureMon(FrStatusTool* stat) {
  return frAddI2CMon(stat, 187, "0xBB Exposure  ", "%3d", 0, 16);
  //return frAddCamPropMon(stat, FR_CHAN_PROP_EXPOSURE_LT, "0xBB Exposure  ", "%3d");
}

int frAddGainMon(FrStatusTool* stat) {
  return frAddI2CMon(stat, 186, "0xBA Gain       ", "%2d", 0, 16);
  //return frAddCamPropMon(stat, FR_CHAN_PROP_GAIN_LT, "0xBA Gain       ", "%2d");
}

int frAddBinMon(FrStatusTool* stat) {
  int ret_val = frAddI2CMon(stat, 165, "0xA5 Desired Bin", "%2d", 0, 16);
  ret_val |= frAddI2CMon(stat, 188, "0xBC Current Bin", "%2d", 0, 16);
  //int ret_val = frAddCamPropMon(stat, FR_CHAN_PROP_DESIRED_BIN_LT, "0xA5 Desired Bin", "%2d");
  //ret_val |= frAddCamPropMon(stat, FR_CHAN_PROP_GAIN_LT, "0xBC Current Bin", "%2d"); // FIXME: no property for current bin set up!
  return ret_val;
}

int frAddAEGCMon(FrStatusTool* stat) {
  //int ret_val = frAddIOCTLMon(stat, FR3_IOC_AEGC_ENABLE_G, "FR3 AEGC       ", "%1d", 0, 1);
  //ret_val |= frAddIOCTLMon(stat, FR3_IOC_AEGC_SKIP_FRAMES_G, "FR3 AEGC SKIP ", "%2d", 0, 16);
  int ret_val = frAddPropertyMon(stat, FR_CHAN_PROP_AEGC_EN, "FR3 AEGC       ", "%1d");
  ret_val |= frAddPropertyMon(stat, FR_CHAN_PROP_AEGC_SKIP, "FR3 AEGC SKIP ", "%2d");
  ret_val |= frAddI2CMon(stat, 175, "0xAF[0] AEC     ", "%1d", 0, 1); // FIXME: no properties for AEC/AGC set up!
  ret_val |= frAddI2CMon(stat, 175, "0xAF[1] AGC     ", "%1d", 1, 1);
  return ret_val;
}

int frAddFPSMon(FrStatusTool* stat, int* frame_count) {
  FrFPSData *fps_data = (FrFPSData *)malloc(sizeof(FrFPSData));
  if (NULL == fps_data) {
    fprintf(stderr, "FR ERROR: frAddFPSMon malloc failed\n");
    return -1;
  }
  fps_data->frameCount = frame_count;

  return frAddCallbackMonitor(stat, fps_monitor, fps_data);
}

static unsigned int masque[] = { 0, 1, 3, 7, 15, 31, 63, 127, 255, 511, 1023,
				 2047, 4095, 8191, 16384, 32768, 65535 };

int frUpdateTrackbars(FrStatusTool* stat); // move function up here later, just writing below to be closer to tbar callbacks for now

int frUpdateMonitors(FrStatusTool* stat) {
  if (stat->n_monitors == 0) 
    return -1;

  cvSetZero(stat->frame);
  char field_text[1024];
  //char text[1024];		       
  char *text = (char*)malloc(1024);
  int  value[2];
  
  for (int i = 0 ; i < stat->n_monitors; i++) {
    FrMonitor *monitor = &stat->monitors[i];

    switch(monitor->type) {
    case FRSTAT_MON_TYPE_I2C: {
      value[0] = frI2CRead(stat->channel, 
			   monitor->field.offset, 
			   0);
      value[1] = frI2CRead(stat->channel, 
			   monitor->field.offset, 
			   1);

      value[0] >>= monitor->field.start;
      value[0] &= masque[monitor->field.size];
      
      value[1] >>= monitor->field.start;
      value[1] &= masque[monitor->field.size];


      sprintf(text, "%s", monitor->field.name);
      strcat(text, " LT-");
      sprintf(field_text, monitor->field.fmt, value[0]);
      strcat(text, field_text);

      strcat(text, " RT-");
      sprintf(field_text, monitor->field.fmt, value[1]);
      strcat(text, field_text);
      break;
    }
    case FRSTAT_MON_TYPE_IOCTL: {
      //if (-1 == ioctl(stat->fd, monitor->field.offset, &value[0]))
      // FIXME: need to replace all ioctl monitors with property monitors!
      value[0] = -1;

      value[0] >>= monitor->field.start;
      value[0] &= masque[monitor->field.size];
      sprintf(text, "%s ", monitor->field.name);
      sprintf(field_text, monitor->field.fmt, value[0]);
      strcat(text, field_text);
      break;
    }

    case FRSTAT_MON_TYPE_CALLBACK: {
      text = monitor->callback(stat, monitor->callback_param);
      break;
    }
    case FRSTAT_MON_TYPE_PROPERTY: {
      sprintf(text, "%s %d", monitor->field.name, 
	      frGetChannelProperty(stat->channel, monitor->field.offset));
      break;
    }
    case FRSTAT_MON_TYPE_CAM_PROP: {
      sprintf(text, "%s LT-%d RT-%d", monitor->field.name, 
	      frGetChannelProperty(stat->channel, monitor->field.offset), 
	      frGetChannelProperty(stat->channel, (monitor->field.offset)+1));
      break;
    }
    }
    
    cvPutText(stat->frame, 
	      text, cvPoint(5, (i+1)*20),
	      &stat->dfont, stat->font_color);
  }
  cvShowImage(stat->wndname, stat->frame);
  free(text);
  frUpdateTrackbars(stat);
  return 0;
}

// This function can be called periodically so that trackbars don't have to have
// a callback function. This function tracks the last values of the trackbars so
// if and only if the value has changed the hardware is updated.
// The last value for each trackbar will have to be held in the FrStatusTool
// struct. Then the current value and last value for every trackbar is there.
int frUpdateTrackbars(FrStatusTool* stat) {
  // Update desired bin if tbar has changed
  if(stat->desired_bin_tbarvalue != stat->last_desired_bin_tbarvalue) {
    stat->last_desired_bin_tbarvalue = stat->desired_bin_tbarvalue;
    if (stat->desired_bin_tbarvalue == 0) stat->desired_bin_tbarvalue = 35;
    //frI2CWriteBC(stat->channel, 165, stat->desired_bin_tbarvalue);
    frSetChannelProperty(stat->channel, FR_CHAN_PROP_DESIRED_BIN, stat->desired_bin_tbarvalue);
  }

  // Update manual exposure if tbar has changed
  if(stat->manual_exp_tbarvalue != stat->last_manual_exp_tbarvalue) {
    stat->last_manual_exp_tbarvalue = stat->manual_exp_tbarvalue;
    if (stat->manual_exp_tbarvalue == 0) {
      // Enable Focus AEGC
      frSetChannelProperty(stat->channel, FR_CHAN_PROP_AEGC_EN, 1);
    } else {
      // Disable Focus AEGC
      frSetChannelProperty(stat->channel, FR_CHAN_PROP_AEGC_EN, 0);
    }
    // Set manual exposure value
    frI2CWriteBC(stat->channel, 11, stat->manual_exp_tbarvalue);
  }

  // Update manual gain if tbar has changed
  if(stat->manual_gain_tbarvalue != stat->last_manual_gain_tbarvalue) {
    stat->last_manual_gain_tbarvalue = stat->manual_gain_tbarvalue;
    if (stat->manual_exp_tbarvalue == 0) {
      // Enable Focus AEGC
      frSetChannelProperty(stat->channel, FR_CHAN_PROP_AEGC_EN, 1);
    } else {
      // Disable Focus AEGC
      frSetChannelProperty(stat->channel, FR_CHAN_PROP_AEGC_EN, 0);
    }
    // Set manual gain value
    frI2CWriteBC(stat->channel, 53, 16+stat->manual_gain_tbarvalue);
  }

  // Update match qual if tbar has changed
  if(stat->match_qual_tbarvalue != stat->last_match_qual_tbarvalue) {
    stat->last_match_qual_tbarvalue = stat->match_qual_tbarvalue;
    frSetChannelProperty(stat->channel, FR_CHAN_PROP_MATCH_QUAL, stat->match_qual_tbarvalue);
  }


  return 0;
}

void frAddDesiredBinTbar(FrStatusTool* stat) {
  cvCreateTrackbar(DESIRED_BIN_TBARNAME, stat->wndname,
		   &stat->desired_bin_tbarvalue, 64, 0);
}

void frAddManualExpTbar(FrStatusTool* stat) {
  cvCreateTrackbar(MANUAL_EXP_TBARNAME, stat->wndname,
		   &stat->manual_exp_tbarvalue, 480, 0);
}

void frAddManualGainTbar(FrStatusTool* stat) {
  cvCreateTrackbar(MANUAL_GAIN_TBARNAME, stat->wndname,
		   &stat->manual_gain_tbarvalue, 48, 0);
}

void frAddMatchQualTbar(FrStatusTool* stat) {
  stat->match_qual_tbarvalue = 64;
  cvCreateTrackbar(MATCH_QUAL_TBARNAME, stat->wndname,
		   &stat->match_qual_tbarvalue, 127, 0);
}


///////////////////////////////////////////////////////////////////////////
/// INTERACTIVE INTERFACES
///////////////////////////////////////////////////////////////////////////
static void read_stdin_int(int *value)
{
	char s1[50];	
	while (1) {
		fgets(s1, 50, stdin);
		
		//printf("<%s> ", s1);
		int retval = sscanf(s1, "%d", value);
		if (retval != 0 &&
		    retval != EOF) {
			break;
			//printf("[%d]\n", *value);
		}
		printf("\nInvalid input<%s>, try again -> ", s1);
	}
}

static void i2c_keypress_handler(FrStatusTool* stat, int keypressed)
{
	
	int offset;
	int devselect;
	int value;

	//int fd = stat->fd;

	if (keypressed == 114) {		// OPTION "r"
		// i2c read
		printf("Enter decimal CSR offset to READ -> ");
		read_stdin_int(&offset);
		printf("Enter sensor to READ (0 master/rt, 1 slave/lt) -> ");
		read_stdin_int(&devselect);
		value = frI2CRead(stat->channel, offset, devselect);
		printf("FR3_IOC_I2C_READ offset=%d, devselect=%d, value=%d\n",
		       offset, devselect, value);
		printf("\n");
	} else if ( keypressed == 119) {	// OPTION "w"
		// i2c write
		printf("Enter decimal CSR offset to WRITE -> ");
		read_stdin_int(&offset);
		printf("Enter sensor to WRITE (0 master/rt, 1 slave/lt) -> ");
		read_stdin_int(&devselect);
		printf("Enter decimal 16bit value to WRITE -> ");
		read_stdin_int(&value);
		frI2CWrite(stat->channel, offset, value, devselect);
		printf("FR3_IOC_I2C_WRITE offset=%d, value=%d, devselect=%d COMPLETE\n",
		       offset, value, devselect);
		printf("\n");
	} else if ( keypressed == 98) {		// OPTION "b"
		// i2c broadcast write
		printf("Enter decimal CSR offset to BROADCAST WRITE -> ");
		read_stdin_int(&offset);
		printf("Enter decimal 16bit value to BROADCAST WRITE -> ");
		read_stdin_int(&value);
		frI2CWriteBC(stat->channel, offset, value);
		printf("FR3_IOC_I2C_WRITE_BC offset=%d, value=%d COMPLETE\n",
		       offset, value);
		printf("\n");
	} else if ( keypressed == 101) {	// OPTION "e"
		// print exposure level
		printf("RT exp: %4d gain %4f  bin %2d %2d    LT exp: %4d gain %4f bin %2d %2d\n", 
		       frI2CRead(stat->channel, 187, 0), ((float)frI2CRead(stat->channel, 186, 0))/16.0f, frI2CRead(stat->channel, 188, 0), frI2CRead(stat->channel, 165, 0),
		       frI2CRead(stat->channel, 187, 1), ((float)frI2CRead(stat->channel, 186, 1))/16.0f, frI2CRead(stat->channel, 188, 1), frI2CRead(stat->channel, 165, 1));
	}
}



void frAddI2CKeypress(FrStatusTool* stat)
{
	printf("\nPress \n" 
	       "\t'r' read i2c\n"
	       "\t'w' write i2c\n"
	       "\t'b' broadcast write i2c\n"
	       "\t'e' read exposure registers\n");

	stat->i2c_notify = 1;
}

int frCheckKeypress(FrStatusTool* stat, int keypressed)
{
	if (-1 != keypressed) {
	        keypressed &= 0xff;
		if (keypressed == 113) // OPTION "q", exit program
			return -1;

		if (stat->i2c_notify)
			i2c_keypress_handler(stat, keypressed);
	}
	return 0;
}



void frCloseStatusTool(FrStatusTool *stat)
{
	cvDestroyWindow(stat->wndname);
	free(stat);
}

