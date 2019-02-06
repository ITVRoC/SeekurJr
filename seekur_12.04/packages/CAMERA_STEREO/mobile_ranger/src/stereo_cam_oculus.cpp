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
  image_transport::Publisher pub = it.advertise("stereo_cam_disp", 1000);
  image_transport::Publisher pub2 = it.advertise("stereo_cam_right", 1000);
  image_transport::Publisher pub3 = it.advertise("stereo_cam_disp_nocolor", 1000);
  image_transport::Publisher pub4 = it.advertise("stereo_cam_left", 1000);
  Mat image, image2, image3, image4;
  sensor_msgs::Image::Ptr msg, msg2, msg3, msg4;
  //sensor_msgs::CvBridge bridge_;
  ros::Rate loop_rate(10);






// codigo opencv_simple_color.c comeca aqui

	FrChannel *channel[3];
	IplImage *frame[3];
	char wndname[3][50];
	

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
	for(i = 0; i < 3; i++) {
	  channel[i] = frOpenChannel(i,device);
		printf("opened channel %d\n",i);
		if (!channel[i]) {
			printf("demo: frOpen failed for device%d\n",i);
			exit(1);			
		}
	}
	

	/* configure the channels as disparity and rt calibrated */
	if(frSetChannelInput(channel[0], FR_CHAN_INPUT_TYPE_DISP)) {
		printf("demo: set INPUT_TYPE failed for device 0\n");
		exit(1);
	}
	if(frSetChannelInput(channel[1], FR_CHAN_INPUT_TYPE_CAL_RT)) {
		printf("demo: set INPUT_TYPE failed for device 1\n");
		exit(1);
	}

	if(frSetChannelInput(channel[2], FR_CHAN_INPUT_TYPE_CAL_LT)) {
		printf("demo: set INPUT_TYPE failed for device 2\n");
		exit(1);
	}


	/* program calibration remap coords */
	char calibfile[128];
	if(device == 0)
		strcpy(calibfile, "/etc/fr/default.calib");
	else
		sprintf(calibfile, "/etc/fr/default%d.calib", device);
	FrCalibMap *map = frOpenCalibMap(calibfile);
	if(frProgRemapData(channel[0], map)) {
		fprintf(stderr, "RemapCoords failed\n");
		exit(1);
	}
	frCloseCalibMap(map);


	/* create display windows for each channel */
	/*sprintf(wndname[0], "Disparity");	
	sprintf(wndname[1], "Right Calib");
	sprintf(wndname[2], "Left Calib");
	cvNamedWindow(wndname[0], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[0], 0, 0);
	cvNamedWindow(wndname[1], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[1], 0, 520);     
	cvNamedWindow(wndname[2], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[2], 0, 520); /*	


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

	FrStatusTool *stat = frOpenStatusTool(channel[0], "Status");
	frAddBinMon(stat);
	frAddExposureMon(stat);
	frAddGainMon(stat);
	frAddAEGCMon(stat);
	frAddFPSMon(stat, &frameCount);

	frAddDesiredBinTbar(stat);
	frAddManualExpTbar(stat);
	frAddManualGainTbar(stat);
	frAddMatchQualTbar(stat);

	frAddI2CKeypress(stat);
	
	//int pixel_mode = frI2CRead(frGetChannelDev(channel[0]), 15, 0);
	//frI2CWriteBC(frGetChannelDev(channel[0]), 15, pixel_mode & (1 << 6));

	// frGrabFrame will start the DMA for each channel 
	for (i = 0; i < 3; i++) { // ALTEREI DE 2 PARA 3
		if(frGrabFrame(channel[i]))
			goto out;
	}



    while (nh.ok())
  {
		
		// frRetreiveFrame will map the DMA buffer to userspace 
		for (i = 0; i < 3; i++) { // ALTEREI DE 2 PARA 3

			frame[i] = frRetrieveFrame(channel[i]);
			if (NULL == frame) 
				goto out;
		}

		// frGrabFrame will start the DMA for each channel 
		for (i = 0; i < 3; i++) { // ALTEREI DE 2 PARA 3
			if(frGrabFrame(channel[i]))
				goto out;
		}		

		// create a colorized disparity image
		frRemapImageColor(frame[0], disp, redLut, greenLut, blueLut);

		/* display the colorized image
		if(showDisp)
		  cvShowImage(wndname[0], disp);

		// display the calibrated image
		if(showCalib)
		  cvShowImage(wndname[1], frame[0]);  */
		
		frameCount++;

		// update monitors every 100 frames
		if(frameCount%10==0) {
		  frUpdateMonitors(stat);
		}

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
		if(-1 == frCheckKeypress(stat, keypressed))
			break;
		    

		//Converte frames iplimage para Mat
		//image = cvarrToMat(disp);
		image2 = cvarrToMat(frame[1]);
		//image3 = cvarrToMat(frame[0]);
		image4 = cvarrToMat(frame[2]);

		//Converte frame Mat para msgs sensor_msgs::Image no ROS    
		//msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(); 		
		msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image2).toImageMsg();
		
		msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image3).toImageMsg();

		msg4 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image4).toImageMsg();

		//Publica as msgs nos topicos  stereo_cam_disp e stereo_cam_right
		//pub.publish(msg);		
		pub2.publish(msg2);
                //pub3.publish(msg3);
		pub4.publish(msg4);

		cv::waitKey(30);  

		ros::spinOnce();
		loop_rate.sleep();


  }

 out:
	frCloseStatusTool(stat);

	/* return ndepth video channels */
	for(i = 0; i < 3; i++) { // ALTEREI DE 2 PARA 3
		frCloseChannel(channel[i]);
	}

  return 0;
}
