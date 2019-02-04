#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <memory>
#include <ros/package.h>

#include "spherical_camera.h"

using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ricoh_capture");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera_360/image_raw/", 1);
  std::string port_;
  nh.getParam("port", port_);
  ROS_INFO("Got param port: %s", port_.c_str());
  std::string rate_;
  nh.param<std::string>("frame_rate", rate_, "14");
  ROS_INFO("Got param frame rate: %s", rate_.c_str());
  VideoCapture cap;
  
  
    // open the default camera using default API
    cap.open(port_);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera " << port_ << "\n";
        return -1;
    }

      std::string path = ros::package::getPath("spherical_camera");
      shared_ptr<SphericalCamera> sphere = shared_ptr<SphericalCamera>(new SphericalCamera(path+"/config/ricoh/front_calib_results.txt",
                                                               path+"/config/ricoh/back_calib_results.txt", 0.03));

  Mat frame;
  ros::Rate loop_rate(atoi(rate_.c_str()));
  while (nh.ok()) {

    cap.read(frame);
    int half = frame.cols/2;

      Mat back = frame(Rect(0,0,half,frame.rows));
        Mat front = frame(Rect(half,0,half,frame.rows));


        rotate(back, back, cv::ROTATE_90_CLOCKWISE);
        rotate(front, front, cv::ROTATE_90_COUNTERCLOCKWISE);

    
    Mat image = sphere->projectOnSphere(front, back);
    



  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
