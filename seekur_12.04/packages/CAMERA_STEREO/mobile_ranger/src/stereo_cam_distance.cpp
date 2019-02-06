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
#include <EllipseDetectorYaed.h>
#include <fstream>

#include "mobile_ranger/Depth.h"

using namespace std;
using namespace cv;

void usageerror()
{
	puts("Usage: openvc_simple_color [-d <device>]\n\n\t-d <device>\tSelect nDepth device (card). 		Default is 0 for first device.");
	exit(2);
}

  CvPoint pt;
void on_mouse( int event, int x, int y, int flags, void* param )
{
	switch ( event )
	{
	case CV_EVENT_LBUTTONDOWN:
	{
		pt = cvPoint(x, y);
	}

	break;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_cam"); 
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("stereo_cam_disp", 1000);
  image_transport::Publisher pub2 = it.advertise("stereo_cam_right", 1000);
  image_transport::Publisher pub3 = it.advertise("stereo_cam_disp_nocolor", 1000);
  ros::Publisher depth_pub = nh.advertise<mobile_ranger::Depth>("depth", 1000);
  Mat image, image2, image3;
  Mat1b gray2;
  sensor_msgs::Image::Ptr msg, msg2, msg3;
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
        
	float focalLength = frGetCameraFocalLength(map);
	float baseline = frGetCameraBaseline(map);

	/* create display windows for each channel */
	sprintf(wndname[0], "Disparity");	
	sprintf(wndname[1], "Right Calib");
	cvNamedWindow(wndname[0], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[0], 0, 0);
	cvNamedWindow(wndname[1], CV_WINDOW_AUTOSIZE);
	cvMoveWindow(wndname[1], 0, 520);     


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

	//----------------------------------------------------------------
	/* used to show distance of pixel selected by the mouse */
	const int r = 4;
	char dispText[1024];
	float distance;
	int   disparity;
	float numerator = focalLength * baseline;

	CvFont dfontMouse;
	CvScalar color = CV_RGB(0, 0, 0);
	CvScalar color2 = CV_RGB(255, 0, 0);
	int line_type = 8;
	float hscaleMouse = .5f;
	float vscaleMouse = .5f;
	int thicknessMouse =2;

	cvInitFont (&dfontMouse, CV_FONT_VECTOR0, hscaleMouse, vscaleMouse, 0.0f, thicknessMouse, line_type);

	// set MouseCallBack
	cvSetMouseCallback(wndname[0], on_mouse,0);
	cvSetMouseCallback(wndname[1], on_mouse,0);


      //-----------------------------------------------------------------------------

	
	//int pixel_mode = frI2CRead(frGetChannelDev(channel[0]), 15, 0);
	//frI2CWriteBC(frGetChannelDev(channel[0]), 15, pixel_mode & (1 << 6));

	// frGrabFrame will start the DMA for each channel 
	for (i = 0; i < 2; i++) {
		if(frGrabFrame(channel[i]))
			{frCloseStatusTool(stat);
			/* return ndepth video channels */
			for(i = 0; i < 2; i++) 
			{frCloseChannel(channel[i]);}
		  	return 0;} 
	}


//--------------------------------------- parametros detec elipse --------------------
	int count = 0;
	//int delay_count = 0;
	float dist_sum = 0;
	float dist_mean = 0;
 	float width_sum = 0;
	float width_mean = 0;
	float hight_sum = 0;
	float hight_mean = 0;
	float dist_vector[5];
	float width_vector[5];
	float hight_vector[5];
	for (int k = 0; k < 5; k++)
		{dist_vector[k]=0;
		 width_vector[k]=0;		
		 hight_vector[k]=0;}
	int width = 752;
	int height = 480;  

	int		iThLength = 16;
	float	fThObb = 3.0f;
	float	fThPos = 1.0f;
	float	fTaoCenters = 0.05f;
	int 	iNs = 16;
	float	fMaxCenterDistance = sqrt(float(width*width + height*height)) * fTaoCenters;

	float	fThScoreScore = 0.4f;

	// Other constant parameters settings. 

	// Gaussian filter parameters, in pre-processing
	Size	szPreProcessingGaussKernelSize = Size(5, 5);
	double	dPreProcessingGaussSigma = 1.0;

	float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
	float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


	// Initialize Detector with selected parameters
	CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
	yaed->SetParameters(szPreProcessingGaussKernelSize,
		dPreProcessingGaussSigma,
		fThPos,
		fMaxCenterDistance,
		iThLength,
		fThObb,
		fDistanceToEllipseContour,
		fThScoreScore,
		fMinReliability,
		iNs
		);
  
// ------------------------------------------------------------------------fim parametros elipse



    while (nh.ok())
  {
		
		// frRetreiveFrame will map the DMA buffer to userspace 
		for (i = 0; i < 2; i++) {

			frame[i] = frRetrieveFrame(channel[i]);
			if (NULL == frame) 
				{frCloseStatusTool(stat);
				/* return ndepth video channels */
				for(i = 0; i < 2; i++) 
				{frCloseChannel(channel[i]);}
			  	return 0;} 

	// Inicio detector de distancia  ------------------------------------------------------
			/* add distance-info (mouse-selection) */
			//create distance-text
			//cvCvtColor( wndname[i], frame[i], CV_GRAY2BGR);    
			cvCircle( frame[i], pt, r+1, color, 1, CV_AA, 0);        
			disparity = ((uchar*)(frame[0]->imageData + frame[0]->widthStep*pt.y))[pt.x];
			
			distance = numerator / disparity;
			sprintf( dispText, "(%d,%d)=%d,%f", pt.x, pt.y, disparity, distance );
     			
			//add distance text to image  
			cvPutText( frame[i], dispText, cvPoint(pt.x+5 ,pt.y+15), &dfontMouse, color);

		}


		// display the frames
		// cvScale(frame[0], disp, 4.0, 0.0);
		/*cvShowImage(wndname[0], disp);
		cvShowImage(wndname[1], frame[1]);*/ 

     // Termino detector distancia   ----------------------------------------------------------

		// frGrabFrame will start the DMA for each channel 
		for (i = 0; i < 2; i++) {
			if(frGrabFrame(channel[i]))
				{frCloseStatusTool(stat);
				/* return ndepth video channels */
				for(i = 0; i < 2; i++) 
				{frCloseChannel(channel[i]);}
			  	return 0;} 
		}		

		// create a colorized disparity image
		frRemapImageColor(frame[0], disp, redLut, greenLut, blueLut);

		/* display the colorized image
		if(showDisp)
		  cvShowImage(wndname[0], disp);

		// display the calibrated image
		if(showCalib)
		  cvShowImage(wndname[1], frame[1]);  */
		
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
		image = cvarrToMat(disp);
		image2 = cvarrToMat(frame[1]);
		image3 = cvarrToMat(frame[0]);


		//Aplica algoritimo de deteccao de circulos  *************************************
			
		vector<Ellipse> ellsYaed;
		gray2 = image2.clone();
		

		// Detecta elipses
		yaed->Detect(gray2, ellsYaed);
		
		// Desenha elipses detectadas
		yaed->DrawDetectedEllipses(image2, ellsYaed);

		// Aponta centro de elipses para detectar distancia

		int sz_ells = int(ellsYaed.size());
		if(sz_ells > 0)
		{ pt.x = ellsYaed[0]._xc;
		  pt.y = ellsYaed[0]._yc;
  		
		// carrega valores de distancia no vetor dist_vector  
		if(count < 5 && disparity != 0)
		     {	dist_vector[count] = distance;
			width_vector[count] = pt.x;
			hight_vector[count] = pt.y;
		        count = count +1;}		

		//ROS_INFO("count : %d", count);
		
		if(count>=5)		
		count = 0;
		}
	
		// calcula media movel de distancia	
		dist_sum = 0;
		dist_mean = 0;
		width_sum = 0;
		width_mean = 0;
		hight_sum = 0;
		hight_mean = 0;

		for (int i = 0; i < 5; i++)
			{dist_sum = dist_vector[i] + dist_sum;
			 width_sum = width_vector[i] + width_sum;	
			 hight_sum = hight_vector[i] + hight_sum;
			}
		  
		dist_mean = dist_sum/5;
		width_mean = width_sum/5;
		hight_mean = hight_sum/5;

		mobile_ranger::Depth depth_msg;

		depth_msg.distance = dist_mean;
		depth_msg.width = width_mean;
		depth_msg.hight = hight_mean;

		/*ROS_INFO("dist_mean : %f", dist_mean);
		ROS_INFO("width_mean : %f", width_mean);
		ROS_INFO("hight_mean : %f", hight_mean);*/
		
 
		//imshow("Yaed", image2);
		
		// termino algoritimo circulos *************************************************


		//cvShowImage(wndname[2], image2);

		//Converte frame Mat para msgs sensor_msgs::Image no ROS    
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(); 		
		msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image2).toImageMsg();
		
		msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image3).toImageMsg();


		//Publica as msgs nos topicos  stereo_cam_disp e stereo_cam_right
		pub.publish(msg);		
		pub2.publish(msg2);
                pub3.publish(msg3);
		depth_pub.publish(depth_msg);
		

		cv::waitKey(30);  

		ros::spinOnce();
		loop_rate.sleep();


  }
/*
 out:
	{frCloseStatusTool(stat);
	/* return ndepth video channels 
	for(i = 0; i < 2; i++) 
	{frCloseChannel(channel[i]);}
  	return 0;} */
}
