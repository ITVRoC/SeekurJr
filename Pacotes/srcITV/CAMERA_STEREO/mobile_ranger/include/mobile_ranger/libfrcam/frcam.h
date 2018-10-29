///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005-2009  Focus Robotics. All rights reserved. 
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
// This file describes the C API to Focus Robotics FR3 Stereo Vision Systems.
//
// For more information on this API, see documentation at: 
// www.FocusRobotics.com/support
//

#ifndef __FRCAM__

#include "cxcore.h"

#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////
/// CAMERA INTERFACE 
///////////////////////////////////////////////////////////////////////////
// Probably want a control interface to access the overall camera
// many of these things could also be done to a channel
// each channel and vcam have a pointer back to the device struct
// This struct may be implicit and not visible to the user.
typedef struct FrCamera FrCamera;
typedef struct FrChannel FrChannel;
typedef struct FrCalibMap FrCalibMap;
typedef struct FrVcam FrVcam;

///////////////////////////////////////////////////////////////////////////
/// CHANNEL INTERFACE 
///////////////////////////////////////////////////////////////////////////
#define FR_CHAN_PROP_FRAME_WIDTH    0
#define FR_CHAN_PROP_FRAME_HEIGHT   1
#define FR_CHAN_PROP_FOURCC         2
#define FR_CHAN_PROP_CHIP_VERSION   3
#define FR_CHAN_PROP_GPIO           4
#define FR_CHAN_PROP_AEGC_EN        5
#define FR_CHAN_PROP_AEGC_SKIP      6
#define FR_CHAN_PROP_DESIRED_BIN    7
#define FR_CHAN_PROP_DESIRED_BIN_LT 8
#define FR_CHAN_PROP_DESIRED_BIN_RT 9
#define FR_CHAN_PROP_EXPOSURE       10
#define FR_CHAN_PROP_EXPOSURE_LT    11
#define FR_CHAN_PROP_EXPOSURE_RT    12
#define FR_CHAN_PROP_GAIN           13
#define FR_CHAN_PROP_GAIN_LT        14
#define FR_CHAN_PROP_GAIN_RT        15
#define FR_CHAN_PROP_MATCH_QUAL     16
#define FR_CHAN_PROP_MAX_SEARCH     17
#define FR_CHAN_PROP_MIN_SEARCH     18
#define FR_CHAN_PROP_foo            19
#define FR_CHAN_PROP_bar            20
#define FR_CHAN_PROP_bas            21

#define FR_CHAN_INPUT_TYPE_DISP		0
#define FR_CHAN_INPUT_TYPE_CAL_RT	1
#define FR_CHAN_INPUT_TYPE_CAL_LT	2
#define FR_CHAN_INPUT_TYPE_RT		3
#define FR_CHAN_INPUT_TYPE_LT		4


/**
 * opens a focus robotics channel for reading/writing
 * @param index channel number (different image input types can be read via different channels, see frSetChannelInput)
 * @param device ndepth/camera device index. Starts at 0, use 1, 2, 3 etc. if the host computer has multiple ndepth devices (ie multiple camera systems).
 * @return FrChannel or NULL if problem opening device
 *
 * A single index sucks for identifying the channel. It kinda makes sense for
 * v4l2, but really you want to identify both the device and the channel within
 * within the device. There really should be a device struct as well that the 
 * channel holds a pointer to.
 *
 * Maybe index should just always be 0-3 with an optional pointer to a FrCamera
 * struct if there are multiple cameras in the system.
 */
//FrChannel* frOpenChannel(int index);
FrChannel* frOpenChannel(int index, int device);

/**
 * attempts to set the specified property
 * @return zero if format accepted
 *
 * Channel properties should be expanded to cover most of the current IOCTLs
 */
int frSetChannelProperty(FrChannel* channel, int property_id, int value);

/**
 * attempts to get the specified property
 * @return property or < 0 if an error occurred
 */
int frGetChannelProperty(FrChannel* channel, int property_id);

/**
 * attempts to set the channel to the specified input type
 * @param index one of FR_INPUT_TYPE_*
 * @return zero if input index accepted
 */
int frSetChannelInput(FrChannel* channel, int index);

/**
 * quickly enqueues a buffer for DMA transfer and returns. 
 * @return < 0 if driver is out of buffers or an error occurred
 */
int frGrabFrame(FrChannel *channel);

/**
 * retrieve the frame grabbed with frGrabFrame. this will block if DMA is
 * not complete yet. if multiple calls to 
 */
IplImage* frRetrieveFrame(FrChannel *channel);

/**
 * steps the hardware processing pipeline. only used when vcam is enabled.
 *
 * I may want to have a type associated with this: one-shot or pipelined or
 * low-latency pipelined.
 */
int frStepVcam(FrChannel* channel);

/**
 * @return the file descriptor for this channel. provided for ioctl 
 * calls.
 * @depricated This call should no longer be necessary. You can now pass the
 * FrChannel pointer everywhere a dev used to be required.
 */
int frGetChannelDev(FrChannel *channel);

/**
 * @return 0 on success
 *
 */
int frProgRemapData(FrChannel *channel, FrCalibMap *map);

/**
 * closes a focus robotics channel device, freeing any resources it was using
 * @param channel device to release
 */
void frCloseChannel(FrChannel *channel);


///////////////////////////////////////////////////////////////////////////
/// CALIBRATION INTERFACE
///////////////////////////////////////////////////////////////////////////

/**
 * opens a file containing undistortion and rectification cooridinates
 * @param filename for the factory supplied calibration information
 * @return FrCalibMap or NULL if a problem opening or reading the file
 */
FrCalibMap* frOpenCalibMap(const char* filename);

/**
 * program a calibration map into the device associated with the the specified
 * file descriptor. this affects all channels associated with the device. 
 * it only needs to be called for one channel per device.
 * @param map containing undistortion and rectification cooridinates.
 * @param channel for the device to load, can be called for any channel
 * @deprecated 
 */
//int frProgramRemapCoords(FrCalibMap *map, int fd); 

/**
 * @deprecated
 */
int frMakeHWRemapCoords(FrCalibMap *map);
int* frGetRemapData(FrCalibMap *map);
int frGetRemapDataLen(FrCalibMap *map);
void frCloseCalibMap(FrCalibMap *map);

/**
 * returns information about the rectified stereo camera
 * @param map containing undistortion and rectification information
 * @return camera baseline in meters
 * @return rectified camera effective focal length in pixels
 */
float frGetCameraBaseline(FrCalibMap *map);
float frGetCameraFocalLength(FrCalibMap *map);

///////////////////////////////////////////////////////////////////////////
/// VIRTUAL CAMERA INTERFACE
///////////////////////////////////////////////////////////////////////////
#define FR_VCAM_PROP_FRAME_WIDTH    0
#define FR_VCAM_PROP_FRAME_HEIGHT   1
#define FR_VCAM_PROP_FOURCC         2

#define FR_VCAM_OUTPUT_TYPE_CAL_RT	0
#define FR_VCAM_OUTPUT_TYPE_CAL_LT	1
#define FR_VCAM_OUTPUT_TYPE_RT		2
#define FR_VCAM_OUTPUT_TYPE_LT		3
#define FR_VCAM_OUTPUT_TYPE_NONE	4


/**
 * opens a focus robotics vcam for reading/writing
 * @param index device in /dev/video%d [4-5]
 * @return FrVcam or NULL if problem opening device
 *
 * A single index is really wrong for this. There should be two values passed.
 * First you want a device number to identify the focus robotics device in case
 * there is more than one in this system. Secondly, you want a vcam channel
 * identifier to show whether you are writing the left or right input channel.
 * The code that talks to v4l should be able to pretty simply turn this into the
 * correct /dev/video device.
 *
 * These calls are all very driver specific.
 */
FrVcam* frOpenVcam(int index);

/**
 * attempts to set the specified property
 * @return zero if format accepted
 */
int frSetVcamProperty(FrVcam* vcam,
		      int property_id, int value);

/**
 * attempts to get the specified property
 * @return property or < 0 if an error occurred
 */
int frGetVcamProperty(FrVcam* vcam, 
		      int property_id);

/**
 * attempts to set the format to match the specified frame
 * @return zero or < 0 if an error occurred
 */
int frSetVcamFmt(FrVcam* vcam,
		 IplImage *frame);

/**
 * attempts to set the vcam to the specified output type. 
 * WARNING: calling this method will enable the vcam and override the
 * camera/calib hardware blocks. 
 * @param index one of FR_INPUT_TYPE_*
 * @return zero if input index accepted
 *
 * Wouldn't this logically be called SetVcamInput? This sets the type of data
 * that we are writing in to this vcam channel.
 */
int frSetVcamOutput(FrVcam* vcam,
		    int index);

/**
 * writes the specified image to hardware for processing.
 * @return zero or < 0 if an error occurred
 */
int frWriteVcamFrame(FrVcam* vcam,
		     IplImage *frame);

/**
 * closes a focus robotics vcam device, freeing any resources it was using
 * @param vcam device to release
 */
void frCloseVcam(FrVcam *vcam);

//////////////////////////////////////////////////////////////////////////////
/// I2C INTERFACE
//////////////////////////////////////////////////////////////////////////////
// All of these should work with an FrChannel pointer rather than a file desc.
// The file descriptor really assumes too much about implementations, whereas
// sending the channel is completely generic and removes a step for the enduser

// These calls are driver specific, but should rarely be needed by the end user.

int frI2CRead(FrChannel *chan, unsigned int offset, unsigned int devselect);
int frI2CWrite(FrChannel *chan, unsigned int offset, unsigned int value, unsigned int devselect);
int frI2CWriteBC(FrChannel *chan, unsigned int offset, unsigned int value);

//////////////////////////////////////////////////////////////////////////////
/// VISUALIZATION ROUTINES
//////////////////////////////////////////////////////////////////////////////
// FIXME: these functions don't have driver dependencies so they are pretty simple to
// change; they should be cleaned up so that the LUT is another blind struct.
void  frCreateColorLUT(int maxvalue, 
		       uchar *redLut, uchar *greenLut, uchar *blueLut);

void frRemapImageColor(IplImage *disp, IplImage *color, 
		       char *redLut, char *greenLut, char *blueLut);

//////////////////////////////////////////////////////////////////////////////
/// MONITORS, GUI TRACKBAR, KEYBOARD INTERACTION, AND OTHER STATUS/CTRL
//////////////////////////////////////////////////////////////////////////////
// FIXME: this interface should be possible to make driver agnostic.
typedef struct FrStatusTool FrStatusTool;
typedef char* (CV_CDECL *FrMonCallback)(FrStatusTool* stat, void* param);

FrStatusTool* frOpenStatusTool(FrChannel* channel, const char* name);
void frCloseStatusTool(FrStatusTool *stat);

int frAddI2CMon(FrStatusTool* stat, int offset, 
		    const char* field_name, const char* field_fmt, 
		    int field_start, int field_size);

int frAddExposureMon(FrStatusTool* stat);
int frAddGainMon(FrStatusTool* stat);
int frAddBinMon(FrStatusTool* stat);
int frAddAEGCMon(FrStatusTool* stat);
int frAddFPSMon(FrStatusTool* stat, int* frame_count);
int frUpdateMonitors(FrStatusTool* stat);


void frAddDesiredBinTbar(FrStatusTool* stat);
void frAddManualExpTbar(FrStatusTool* stat);
void frAddManualGainTbar(FrStatusTool* stat);
void frAddMatchQualTbar(FrStatusTool* stat);
void frAddI2CKeypress(FrStatusTool* stat);
int  frCheckKeypress(FrStatusTool* stat, int keypressed);




//////////////////////////////////////////////////////////////////////////////
/// IOCTL DEFINES
//////////////////////////////////////////////////////////////////////////////
// FIXME:
// The FrChannel should really have functions to accomplish all of these read
// and write operations. Everything should be wrapped and we should not assume
// that an IOCTL call will be used at all. This section of the header should be
// removed for this interface to really be generic.
//
// A lot of this stuff is really device orientated. Maybe the interface should
// be expanded to have an frcam at the top which in turn holds multiple frchans
// and frvcams. The frcam would mostly pop into existance automagically when a
// channel is opened to keep things backward compatible.
#define LINUX_V4L2
#ifdef LINUX_V4L2
#include <sys/ioctl.h>
#define FR3_IOC_MAGIC 0xF4

#define FR3_IOC_CHIP_VERSION		_IO(FR3_IOC_MAGIC, 1)
#define FR3_IOC_PROGRAM_REMAP_COORDS    _IO(FR3_IOC_MAGIC, 2)
#define FR3_IOC_GPIO_READ               _IO(FR3_IOC_MAGIC, 3)
#define FR3_IOC_GPIO_WRITE              _IO(FR3_IOC_MAGIC, 4)
#define FR3_IOC_STEP_HARDWARE		_IO(FR3_IOC_MAGIC, 5)
#define FR3_IOC_I2C_WRITE		_IO(FR3_IOC_MAGIC, 6)
#define FR3_IOC_I2C_WRITE_BC		_IO(FR3_IOC_MAGIC, 7)
#define FR3_IOC_I2C_READ		_IO(FR3_IOC_MAGIC, 8)
#define FR3_IOC_AEGC_ENABLE_S		_IO(FR3_IOC_MAGIC, 9)
#define FR3_IOC_AEGC_ENABLE_G		_IO(FR3_IOC_MAGIC, 10)
#define FR3_IOC_AEGC_SKIP_FRAMES_S	_IO(FR3_IOC_MAGIC, 11)
#define FR3_IOC_AEGC_SKIP_FRAMES_G	_IO(FR3_IOC_MAGIC, 12)
#define FR3_IOC_MQC_S			_IO(FR3_IOC_MAGIC, 13)
#define FR3_IOC_MQC_G			_IO(FR3_IOC_MAGIC, 14)

//#define FR_DEBUG 1

/// Debug printing
#undef PDEBUG             /* undef it, just in case */
#ifdef FR_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "%s: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#endif


  //////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif

#endif
