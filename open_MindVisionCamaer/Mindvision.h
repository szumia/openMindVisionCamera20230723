#ifndef MINDVISION_H
#define MINDVISION_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc_c.h>
#include <CameraApi.h>
#include <chrono>
#include <thread>

using namespace std;
#include <string>

//拍照间隔 2s
#define INTERVAL 2
#define RATE     300
#define PHOTOSNUM 250
#define CLICK_DOWN 5

namespace MindVision {

class Mindvision
{
public:
    ~Mindvision();
    void init_camera(int tigger_mode=0);
    void get_image();
    void control_camera(int camera_mode=0);                 //0 拍照并处理   1 只拍照

    tSdkCameraCapbility   tCapability;
    tSdkFrameHead         sFrameInfo;
    unsigned char *       g_pRgbBuffer=NULL;
    BYTE*                 pybBuffer=NULL;
    int                   camera_handle ;
    int                   channel =3;
    cv::Mat                   src;

    //不处理，只拍照
    void get_photos_only();
    void get_image_automatic(int camera_mode=1);

    //相机读图处理一条龙
    void camera_deal_img();
};




}














#endif // MINDVISION_H
