#include "pnp_detect.h"


void PnpDetect::pnp_detect(cv::Mat orignal,armor_detect::Armor &armor_pnp)
{
    //装甲板世界坐标————左下逆时针
    this->worldpoints.clear();
    worldpoints.push_back(Point3f(-67.5f,28.5f,0));
    worldpoints.push_back(Point3f(67.5f,28.5f,0));
    worldpoints.push_back(Point3f(67.5f,-28.5f,0));
    worldpoints.push_back(Point3f(-67.5f,-28.5f,0));
    // 2D 特征点像素坐标---必须和世界坐标一一对应
    this->imagepoints.clear();
    for(int i=0;i<(int)armor_pnp.out_points.size();i++)
    {
        imagepoints.push_back(armor_pnp.out_points[i]);
    }

    //绘制出标定点
    for(int i=0;i<4;i++){
        putText(orignal,to_string(i+1),imagepoints[i],1,1,Scalar(0,255,0));
    }

    solvePnP(worldpoints,imagepoints,cameraMatrix,distCoeffs,R,T,0);
    //转换格式
    R.convertTo(Rvec,CV_32F);
    T.convertTo(Tvec,CV_32F);
    //方法一：
    double dis1= sqrt(T.at<double>(0)*T.at<double>(0)+T.at<double>(1)*T.at<double>(1)+T.at<double>(2)*T.at<double>(2));
    double yaw=atan(T.at<double>(0)/T.at<double>(2))*180/3.14;
    double pitch=-atan(T.at<double>(1)/T.at<double>(2))*180/3.14;
    armor_pnp.pnp_dis=dis1;
    armor_pnp.yaw=yaw;
    armor_pnp.pitch=pitch;
    cout<<"dis        :"<<"【"<<dis1<<"】"<<endl;
//    cout<<"yaw        :"<<"【"<<yaw<<"】"<<endl;
//    cout<<"pitch      :"<<"【"<<pitch<<"】"<<endl;

//    //方法二：不科学
//    Mat_<float> rotMat(3,3);
//    Rodrigues(Rvec,rotMat);
//    //正确的是根号下x^2+y^2+z^2,直接用上面的Tvec的【0】【1】【2】下面的误差刚好抵消反而误差更小
//    Mat p;
//    p=-rotMat.inv()*Tvec;
//    cout<<"dis_oc     :"<<"【"<<p.at<double>(2)<<"】"<<endl;
}
