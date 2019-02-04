
/*
 * Spherical Cam.h
 *
 *  Created on: Aug 20, 2018
 *      Author: Daniel Balbino de Mesquita
 */


#ifndef SPHERECAM_H_
#define SPHERECAM_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "omni_camera.h"


class SphericalCamera {//: public AbstractCamera {


private:

float rho_limit;
float baseline;

public:

  SphericalCamera(std::string calibFile1, std::string calibFile2, float baseline =0);
  ~SphericalCamera();


  void create_spherical_proj( cv::Mat *mapx, cv::Mat *mapy, OmniCamera cam, float plus_theta =0, float zi =0, const float rho_limit= M_PI/2) ;

  cv::Mat simpleBlend(const cv::Mat &front,const  cv::Mat &back);
  bool controlParameters(char key);
  Vector3d toSphereRad(float theta, float phi);
  Vector3d toSphere(float theta, float phi);

  cv::Mat projectOnSphere(cv::Mat frontImage, cv::Mat backImage, int blend_type =0);

  cv::Mat mapx_f;
  cv::Mat mapy_f;
  cv::Mat mapx_b;
  cv::Mat mapy_b;
  OmniCamera frontCamera;
  OmniCamera backCamera;

  


} ;

#endif 


