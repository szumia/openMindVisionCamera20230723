#ifndef PNP_DETECT_H
#define PNP_DETECT_H
#include<opencv2/opencv.hpp>
#include "armor_detect.h"

using namespace std;

class PnpDetect
{
public:
     //内参   opencv测得
//  Mat cameraMatrix=(cv::Mat_<double>(3,3)<<2035.450632415266, 0, 589.5698032914486,
//          0, 2058.543303892454, 127.5492345143006,
//          0, 0, 1);
    //matlab测得
  Mat cameraMatrix=(cv::Mat_<double>(3,3)<<2049.75664228553,0,718.643901637294,
                    0,2019.41742419379,108.130476334550,
                    0,0,1);
    //畸变
//  Mat distCoeffs=(cv::Mat_<double>(5,1)<<-0.08487740187879322, 0.06456759412168639, -0.005402572106722016,
//                  -6.112157820600937e-05, 1.048228247636774);
  Mat distCoeffs=(cv::Mat_<double>(5,1)<<0,0,0,0,0);
  // 2D 特征点像素坐标
  vector<cv::Point2d>               imagepoints;
  //裝甲板世界坐标
  vector<cv::Point3d>               worldpoints;
  //R 旋转向量    T 平移向量
  cv::Mat                           R,T;
  cv::Mat                           Rvec;
  cv::Mat_<float>                   Tvec;
  PnpDetect(){};
  void pnp_detect(cv::Mat orignal,armor_detect::Armor &armor_pnp);

};


#endif // PNP_DETECT_H

