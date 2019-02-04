/*
 */

/*
 * 
 *
 *  Created on: Aug 18, 2018
 *      Author: Daniel Balbino de Mesquita
 */

#include <stdio.h>
#include <math.h>
#include <iostream>
#include "spherical_camera.h"
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>




using namespace cv;
using namespace std;

SphericalCamera::SphericalCamera( std::string calibFile1, std::string calibFile2, float baseline ):frontCamera(calibFile1), backCamera(calibFile2)
{

    
    rho_limit = M_PI/2+0.01;


     
     this->mapx_f  = cv::Mat(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_32FC1);
     this->mapy_f  = cv::Mat(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_32FC1);

     this->mapx_b  = cv::Mat(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_32FC1);
     this->mapy_b  = cv::Mat(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_32FC1);


    create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline, rho_limit);
    create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);


}

Vector3d SphericalCamera::toSphere(float theta, float phi){

      Vector3d out;


      out[0] = sin(theta *M_PI/180) * cos(phi*M_PI/180);

      out[1] = sin(phi *M_PI/180);
      out[2] = cos(theta *M_PI/180)* cos(phi*M_PI/180);

 
      return out;

}


Vector3d SphericalCamera::toSphereRad(float theta, float phi){

      Vector3d out;

      out[0] = sin(theta ) * cos(phi);
      out[1] = sin(phi);
      out[2] = cos(theta )* cos(phi);

      return out;

}



void SphericalCamera::create_spherical_proj( cv::Mat *mapx, cv::Mat *mapy, OmniCamera cam, float plus_theta, float zi, const float rho_limit )
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     Eigen::Vector2d m;

     float theta, phi;
     float step_theta = 2.0*M_PI/width;
     float step_phi = M_PI/height;


     *mapx = 0;
     *mapy = 0;

     cout << rho_limit << endl;
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {

             theta = j * step_theta - M_PI+plus_theta;
             phi = i * step_phi - M_PI/2.0;

             Vector3d dir = toSphereRad(theta,phi);

             float rho = acos(dir[2]);

             dir[2]+=zi;

            if (rho<(rho_limit)){
             Eigen::Vector2d m (cam.world2cam(dir));
             mapx->at<float>(i,j) = (float) m[0];
             mapy->at<float>(i,j) = (float) m[1];
            }
            else{
             mapx->at<float>(i,j) = (float) -1;
             mapy->at<float>(i,j) = (float) -1;
            }
         }
}





cv::Mat SphericalCamera::simpleBlend(const cv::Mat &front,const  cv::Mat &back){

    Mat f_gray, b_gray;
    if (front.channels() == 3){
        cvtColor(front, f_gray, cv::COLOR_BGR2GRAY);
    }
    else{
        f_gray= front.clone();
    }
    if (back.channels() == 3){
        cvtColor(back, b_gray, cv::COLOR_BGR2GRAY);
    }
    else{
        b_gray= back.clone();
    }

    Mat maskFirst = f_gray > 0;
    Mat maskSecond = b_gray > 0;

    Mat intersect;
    cv::multiply(maskFirst, maskSecond, intersect);
    

    Mat result, seam;

    cv::add(front/2, back/2, seam, intersect>0);

    cv::add(front, back, result, intersect==0);

    cv::add(seam, result, result, intersect>0);
     
    return result;
}

cv::Mat SphericalCamera::projectOnSphere(cv::Mat frontImage, cv::Mat backImage,int blend_type){
        Mat sphere1 = cv::Mat::zeros(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_8UC3);
        Mat sphere2 = cv::Mat::zeros(this->frontCamera.ocamModel.height, 2*this->frontCamera.ocamModel.height, CV_8UC3);
        cv::remap( frontImage, sphere1, mapx_f, mapy_f, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT, cv::Scalar(0,0, 0) );
        cv::remap( backImage, sphere2, mapx_b, mapy_b, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT, cv::Scalar(0,0, 0) );

        

        return simpleBlend(sphere1, sphere2);
    
}


bool SphericalCamera::controlParameters(char key){

    bool finish;
    switch(key) {
             case 'w' :
                baseline+=0.01;
                create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline,rho_limit);
                create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);
                cout << baseline << endl;
                break;

             case 's' :
                baseline-=0.01;
                create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline,rho_limit);
                create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);

                cout << baseline << endl;
                break;
            case 'e' :
               baseline+=0.001;
               create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline,rho_limit);
               create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);
               cout << baseline << endl;
               break;

            case 'd' :
                baseline-=0.001;
                create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline,rho_limit);
                create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);

                cout << baseline << endl;
               break;


            case 'q' :
               rho_limit+=0.01;
               create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline,rho_limit);
               create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);
               cout << rho_limit << endl;
               break;

            case 'a' :
                rho_limit-=0.01;
                create_spherical_proj(&this->mapx_f,&this->mapy_f, this->frontCamera,0, baseline, rho_limit);
                create_spherical_proj(&this->mapx_b,&this->mapy_b, this->backCamera, M_PI,baseline,rho_limit);

               cout << rho_limit << endl;
               break;

            case 'z' :
                finish = true;
                break;





             default :
                break;


          }
    return finish;
}




SphericalCamera::
~SphericalCamera()
{}

