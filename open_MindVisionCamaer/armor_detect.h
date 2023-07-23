#ifndef ARMOR_DETECT_H
#define ARMOR_DETECT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Core>

using namespace  std;
using namespace cv;

//灯条筛选
#define GRAYBOUND       80
#define BOUND           40     //用于二值化的低阈值
#define CONTOURMIN      20
#define CONTOURMAX      12000
#define MAXRATIO        12.0   //宽高比
#define MINRATIO        1.0
#define MINTUDU         0.45  //凸度·
#define MINANGLE        50
#define MAXANGLE        130

//装甲板初筛选
#define MAXANGLELANDR      0.4
#define MAXPARALLELISM     0.1
#define MAXHCOMHLANDR      1.8
#define MINWCOMHLANDR      1.5
#define MAXWCOMHLANDR      6.0

//装甲板筛选
#define MINARMORRECTAREA     300
#define MAXARMORHCOMW        6
#define MINARMORANGLE        50
#define MAXARMORANGLE        130

namespace armor_detect {
bool comPoint(const Point2f & p1,const Point2f & p2);
bool comRotateRect(RotatedRect &p1,RotatedRect &p2);
double get_ous_distance(const Point2f &p1,const Point2f &p2);
double get_vec_angle(const Point2f &p1,const Point2f &p2);

class Light
{
public:
    double angle;     //x轴顺时针旋转为准  靠近长边为标准
    double hCompw;
    double tudu;
    double height;
    Point2f center;
    Point2f light_vec;         //向量边
    Mat img;
    vector<Point2f> contour;
    vector<Point2f> points;    //按左下-右下-左上-右上  入栈
    RotatedRect light_rect;

    Light(){};
    Light(vector<Point> contour);
    Light(RotatedRect &light_rect);
    vector<Point2f> get_light_points()   const
    {
        return  this->points;
    }
};

class Armor
{
public:
    //左右灯条匹配时的一些属性
    double angle1,angle2;
    double weight,height_ave;
    double angle_LandR;
    double WcomH_LandR;
    double HcomH_LandR;
    double parallelism;
    double angle_ratio;
    int    left_index,right_index;      //左右灯条下标号
    int    armor_filter_index;

    //装甲板筛选的属性
    double armor_HcomW;                 //装甲板宽高比
    double armor_width;
    double armor_height;
    double armor_rect_angle;            //装甲板角度
    double armor_rect_area;

    //pnp得到的装甲板属性
    double yaw;
    double pitch;
    double pnp_dis;

    Light           left_light_armor,right_light_armor;
    vector<Point2f> contours_points;    //轮廓四点
    vector<Point2f> out_points;         //装甲板外四点——真实装甲板    左下逆时针
    RotatedRect     armor_rect;             //装甲板矩形

    Armor(Light & left,Light& right);
    Armor(){};
    void draw_armor(Mat &orignal,Mat &frame);
};

class Detect
{
public:
    vector<RotatedRect> lightInfos;                         //存初筛后的灯条
    vector<Armor> armor_filter;                             //存初筛后的装甲板
    Detect(){}
    void preDeal(Mat & frame,int color);
    void contoursDeal(Mat & frame);
    void matchLight(vector<RotatedRect> &lightInfos,Mat & frame,Mat & orignal);
    void matchLight_deeper();

    //得出装甲板流程封装函数
    void detect_steps(Mat &frame,Mat & orignal,int color);
};

}






#endif //ARMOR_DETECT_H



