#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <memory>

#include "spherical_camera.h"

using namespace cv;
using namespace std;


using namespace sensor_msgs;
using namespace message_filters;

cv::Mat cv_inicial, cv_zoom, cv_imagem;

shared_ptr<SphericalCamera> sphere;


void callback(const ImageConstPtr& image1, const ImageConstPtr& image2 ,image_transport::Publisher& image_pub_){

    cv_bridge::CvImagePtr cv_ptr1; 
    cv_bridge::CvImagePtr cv_ptr2; 

    ROS_INFO("Callback reached! :-)");


        try
        {
          cv_ptr1    = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
          cv_ptr2    = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("\ncv_bridge exception: %s", e.what());
          return;
        }

        

        Mat panorama = sphere->projectOnSphere(cv_ptr1->image,cv_ptr2->image);

        cv::imshow("SphereView", panorama);
        cv::waitKey(30);


        //image_pub_.publish(cv_ptr1->toImageMsg());
  }

        
        
    
 

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
     try
     {
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
   

class ImageConverter
{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    message_filters::Subscriber<Image> image1_sub_, image2_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr;
  
   
    public:
    
    ImageConverter(): it_(nh_)
    {

      std::string path = ros::package::getPath("spherical_camera");
      sphere = shared_ptr<SphericalCamera>(new SphericalCamera(path+"/config/front_calib_results.txt",
                                                               path+"/config/back_calib_results.txt"));

      cv::namedWindow("SphereView");
       cv::startWindowThread();

      


      

      //image_transport::Subscriber sub = it_.subscribe("/camera_360_f/image_raw_front", 1, imageCallback);
      image1_sub_.subscribe(nh_,"/camera_360_f/image_raw_front",1);
      image2_sub_.subscribe(nh_,"/camera_360_b/image_raw_back",1 );
      image_pub_ = it_.advertise("/camera_360/image_raw", 1);

      typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
      
      Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub_, image2_sub_);
      sync.registerCallback(boost::bind(&callback, _1, _2, boost::ref(image_pub_)));//);

      
      image_pub_ = it_.advertise("/camera_360/image_raw", 1);
      //image_sub_ = it_.subscribe("/camera_360_front/image_raw_front", 1, &ImageConverter::imageCb, this);
      ros::spin();
    }


    



};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_360");
  ImageConverter ic;
        
  return 0;
}
