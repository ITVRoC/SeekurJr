#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

#include <sstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include <frcam.h>
#include <cv_bridge/cv_bridge.h>
//#include "CvBridge.h"
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;

void usageerror()
{
	puts("Usage: openvc_simple_color [-d <device>]\n\n\t-d <device>\tSelect nDepth device (card). 		Default is 0 for first device.");
	exit(2);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_cam"); 
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub2 = it.advertise("stereo_cam_right", 1000);
  Mat image, image2, image3, image4;
  sensor_msgs::Image::Ptr msg, msg2, msg3, msg4;
  //sensor_msgs::CvBridge bridge_;
  ros::Rate loop_rate(10);

// codigo opencv_simple_color.c comeca aqui

	FrChannel *channel;
	IplImage *frame;
	char wndname[50];
	

	int i;
	int device = 0;

	for(i = 1; i < argc; ++i)
	{
        	if(strcmp(argv[i], "-d") == 0)
           	{
			if(argc > i)
				device = atoi(argv[++i]);
			else
				usageerror();
	   	}
		else
		{
			usageerror();
		}
	}
	  
	printf("device %d\n", device);
 	  

	/* get unconfigured ndepth video channels, device supports up to 4 */
	channel = frOpenChannel(0,device);
	printf("opened channel %d\n",0);
	if (!channel) {
		printf("demo: frOpen failed for device%d\n",0);
		exit(1);			
	}
	
	

	/* configure the channels as disparity and rt calibrated */
	
	if(frSetChannelInput(channel, FR_CHAN_INPUT_TYPE_CAL_RT)) {
		printf("demo: set INPUT_TYPE failed for device 1\n");
		exit(1);
	}


	/* program calibration remap coords */
	char calibfile[128];
	if(device == 0)
		strcpy(calibfile, "/etc/fr/default.calib");
	else
		sprintf(calibfile, "/etc/fr/default%d.calib", device);
	FrCalibMap *map = frOpenCalibMap(calibfile);
	if(frProgRemapData(channel, map)) {
		fprintf(stderr, "RemapCoords failed\n");
		exit(1);
	}
	frCloseCalibMap(map);


	/* used for color image */
	IplImage *disp = cvCreateImage(cvSize(752, 480), IPL_DEPTH_8U, 3); // FIXME: don't hardcode image size
	const int maxDisparityValue = 63;	
	char *redLut = (char*)malloc(maxDisparityValue);
	char *greenLut = (char*)malloc(maxDisparityValue);
	char *blueLut = (char*)malloc(maxDisparityValue);
	frCreateColorLUT(maxDisparityValue, (uchar*)redLut, (uchar*)greenLut, (uchar*)blueLut);

	/* add status/control interfaces */
	int keypressed;
	int frameCount = 0;
	int showDisp = 1;
	int showCalib = 1;

	
	
	//int pixel_mode = frI2CRead(frGetChannelDev(channel[0]), 15, 0);
	//frI2CWriteBC(frGetChannelDev(channel[0]), 15, pixel_mode & (1 << 6));


  while (nh.ok())
  {
		
		// frRetreiveFrame will map the DMA buffer to userspace 
		
		frame = frRetrieveFrame(channel);
		
		// frGrabFrame will start the DMA for each channel 
		if(frGrabFrame(channel))
			goto out;
		
		// create a colorized disparity image
		frRemapImageColor(frame, disp, redLut, greenLut, blueLut);

		
		frameCount++;

		
		/* check pressed key, if any */
		keypressed = cvWaitKey(2);
		if(keypressed=='d') {
		  showDisp = !showDisp;
		  if(showDisp) printf("Disparity display is enabled\n");
		  else printf("Disparity display is disabled\n");
		} else if(keypressed=='c') {
		  showCalib = !showCalib;
		  if(showDisp) printf("Calibrated right image display is enabled\n");
		  else printf("Calibrated right image display is disabled\n");
		}
		

		//Converte frames iplimage para Mat
		image2 = cvarrToMat(frame);
		
		//Converte frame Mat para msgs sensor_msgs::Image no ROS    
		msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image2).toImageMsg();
		
		//Publica as msgs nos topicos  stereo_cam_disp e stereo_cam_right
		pub2.publish(msg2);
                
                cv::waitKey(30);  

		ros::spinOnce();
		loop_rate.sleep();


  }
out:
 	/* return ndepth video channels */
	frCloseChannel(channel);

  return 0;
}
