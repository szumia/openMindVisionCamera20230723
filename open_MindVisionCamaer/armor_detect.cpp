#include "armor_detect.h"
#include "pnp_detect.h"

using Light = armor_detect::Light;
using Armor = armor_detect::Armor;
using Detect = armor_detect::Detect;

bool armor_detect::comPoint(const Point2f & p1,const Point2f & p2)
{
    return p1.y>p2.y;
}

//按灯条中心点升序
bool armor_detect::comRotateRect(RotatedRect &p1,RotatedRect &p2)
{
    return p1.center.x< p2.center.x;
}

//获取两点间欧拉距离
double armor_detect::get_ous_distance(const Point2f &p1,const Point2f &p2)
{
    double x2=pow((p1.x-p2.x),2);
    double y2 = pow((p1.y-p2.y),2);
    double dis =sqrt(x2+y2);
    return dis;
}

//获取两向量坐标夹角
double armor_detect::get_vec_angle(const Point2f &p1,const Point2f &p2)
{
    double dot = p1.x*p2.x+p1.y*p2.y;
    double c1=get_ous_distance(p1,Point2f(0,0));
    double c2=get_ous_distance(p2,Point2f(0,0));
    double rad = dot/c1/c2;
    if(rad>1.0)rad=1;
    else if(rad<-1.0)rad=-1;
    return acos(rad);
}

bool operator==(const Light& one,const Light& other)
{
    for(int i=0;i<4;i++)
    {
        if(one.get_light_points()[i]!=other.get_light_points()[i])
        {
            return false;
        }
    }
    return  true;
}

Light::Light(vector<Point> contour)
{
    //椭圆拟合区域得到外接矩形   角度  轮廓多于5个点才能用fitEllipse
    if(contour.size()>5)
    {
    this->light_rect = fitEllipse(contour);
    }

    //角度---------用宽高值区分方向---------可能出问题
    //4.5版本定义为，x轴顺时针旋转最先接触的边的延长线为w，angle为x轴顺时针旋转的角度，angle取值为(0,90]
    //rect[0]返回最小外接矩形的中心点，rect[1]为最小外接矩形的宽、高。rect[2]为旋转角度。

    if(light_rect.size.width>light_rect.size.height)
    {
        angle = light_rect.angle;
        //cout<<angle<<"  ";
    }
    else
    {
        angle = 180.0 - light_rect.angle;
        //cout<<angle<<"  ";
    }
//    cout<<angle<<"  ";

    //宽高比
    hCompw = 1.0*light_rect.size.height/light_rect.size.width;
    if(hCompw<1)hCompw = 1.0/hCompw;

    //凸度-----椭圆面积/矩形面积来衡量
    tudu = 1.0*contourArea(contour)/light_rect.size.area();
}


Light::Light(RotatedRect &light_rect)
{
    this->light_rect = light_rect;
    if(light_rect.size.width>light_rect.size.height)
    {
        angle = light_rect.angle;
    }
    else
    {
        angle = 180.0 - light_rect.angle;
    }

    //宽高比
    hCompw = 1.0*light_rect.size.height/light_rect.size.width;
    if(hCompw<1.0)hCompw = 1.0/hCompw;
    //cout<<hCompw<< " ";

    points.clear();
    //四个顶点坐标
    Point2f dingdian[4];
    vector<Point2f> four_points;
    light_rect.points(dingdian);
    for(int i=0;i<4;i++)
    {
        four_points.push_back(dingdian[i]);
    }

    //对四个y坐标进行了降序排序    y值大，在图像是越下
    sort(four_points.begin(),four_points.end(),comPoint);
    if(four_points[0].x<four_points[1].x)
    {
        points.push_back(four_points[0]);
        points.push_back(four_points[1]);
    }
    else {
        points.push_back(four_points[1]);
        points.push_back(four_points[0]);
    }

    if(four_points[2].x<four_points[3].x)
    {
        points.push_back(four_points[2]);
        points.push_back(four_points[3]);
    }
    else {
         points.push_back(four_points[3]);
         points.push_back(four_points[2]);
    }

    //矩形长度
    height=get_ous_distance(points[0],points[2]);

    //中心
    center = light_rect.center;

    //向量边坐标
    light_vec=points[2]-points[0];

}

Armor::Armor(Light & left,Light& right)
{
    this->left_light_armor=left;
    this->right_light_armor=right;
    angle_LandR=abs((left.center.y-right.center.y)/(left.center.x-right.center.x));
    angle_LandR=atan(angle_LandR);
    weight=get_ous_distance(left.center,right.center);
    height_ave=(left.height+right.height)/2.0;
    WcomH_LandR=weight/height_ave;
    HcomH_LandR=left.height/right.height;
    if(HcomH_LandR<1)
    {
        HcomH_LandR=1.0/HcomH_LandR;
    }
    parallelism=get_vec_angle(left.light_vec,right.light_vec);
//angle_ratio用不到
//        angle1=abs((left.points[0].y-right.points[0].y)/(left.points[0].x-right.points[0].x));
//        angle2=abs((left.points[1].y-right.points[1].y)/(left.points[1].x-right.points[1].x));
//        angle_ratio=angle1/angle2;

}

void Armor::draw_armor(Mat &orignal,Mat &frame)
{
//            cout<<"00000000"<<endl;
//            rectangle(frame,left.points[2],right.points[1],Scalar(255));
//            rectangle(orignal,left.points[2],right.points[1],Scalar(0,255,0));

    for(int i=0;i<(int)this->out_points.size();i++)
    {
        if(i==3)line(frame,out_points[3],out_points[0],Scalar(255));
        else line(frame,out_points[i],out_points[i+1],Scalar(255));
    }

    for(int i=0;i<(int)this->out_points.size();i++)
    {
        if(i==3)line(orignal,out_points[3],out_points[0],Scalar(255,255,255));
        else line(orignal,out_points[i],out_points[i+1],Scalar(255,255,255));

    }
}

void Detect::preDeal(Mat & frame,int color)
{
    Mat gray_mat,green_mat;
    vector<Mat> channels;
    split(frame,channels);
    if(color == 1)        //蓝色
    {
        gray_mat = channels.at(0)-channels.at(2);
        threshold(channels[1],green_mat,GRAYBOUND,255,THRESH_BINARY);
    }
    else if(color ==2)   //红色
    {
        gray_mat = channels.at(2)-channels.at(0);
        threshold(channels[1],green_mat,GRAYBOUND,255,THRESH_BINARY);
    }

    //去噪
    GaussianBlur(gray_mat,gray_mat,Size(3,3),0,0);
    //二值化
    Mat bin_mat;
    threshold(gray_mat,bin_mat,BOUND,255,THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(3,3));    //3*3椭圆卷积核
    //先膨胀  （两次也没啥用）
    dilate(bin_mat,bin_mat,kernel);
    //与灰度二值化图取并集----筛选掉亮度低的颜色
    bin_mat = green_mat & bin_mat;
    frame = bin_mat;
    //imshow("预处理",frame);

}
void Detect::contoursDeal(Mat & frame)
{
    //找出所有轮廓存储在light_contours
    vector<vector<Point>> light_contours;
    findContours(frame,light_contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
//     //填充轮廓---高曝光？ 没啥用
//    drawContours(frame,light_contours,0,Scalar(255),FILLED);
//    findContours(frame,light_contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //拟合对象配对筛选
    for(const auto& lightcontour :light_contours)
    {
        double contour_area = contourArea(lightcontour);
        // cout<<contour_area<<" ";

        //面积太大或者太小，舍弃
        if(contour_area<CONTOURMIN||contour_area>CONTOURMAX||lightcontour.size()<6)
            continue;

          Light light(lightcontour);
//        //根据灯条属性筛选轮廓
//            cout<<light.tudu<<" ";
         if(light.tudu<MINTUDU)continue;
//           cout<<light.tudu<<" ";

//           cout<<light.hCompw<< "  ";
         if(light.hCompw>MAXRATIO||light.hCompw<MINRATIO)continue;

         if((light.angle>=MINANGLE&&light.angle<=MAXANGLE))continue;
//           cout<<light.angle<<" ";

        this->lightInfos.push_back(light.light_rect);
    }

}

//增强匹配条件
void Detect::matchLight_deeper()
{
    //装甲板中间不可能还有灯条
    vector<int> armor_filter_order_vector;
    for(int i=0;i<(int)armor_filter.size();i++)
    {
        armor_filter[i].armor_filter_index=i;
    }

    for(int i=0;i<(int)armor_filter.size()-1;i++)
    {
        for(int j=0;j<(int)armor_filter.size();j++)
        {
           if(armor_filter[i].right_light_armor==armor_filter[j].left_light_armor)
           {
              armor_filter_order_vector.push_back(armor_filter[j].armor_filter_index);
           }
        }
        //删除装甲板中间的灯条
        for(int i=0;i<(int)armor_filter_order_vector.size();i++)
        {
            this->armor_filter.erase(armor_filter.begin()+armor_filter_order_vector[i]);
        }
      armor_filter_order_vector.clear();
    }
}

//灯条匹配---得到符合条件装甲版
void Detect::matchLight(vector<RotatedRect> &lightInfos,Mat & frame,Mat & orignal)
{
    if(lightInfos.empty())return;
    if(lightInfos.size()<2||lightInfos.size()>15)
    {
        return;
    }

    sort(lightInfos.begin(),lightInfos.end(),comRotateRect);

    //cout<<"light: "<<lightInfos.size()<<endl;
    //框出筛选的灯条
//    for(int i=0;i<lightInfos.size();i++){
//        auto rect=lightInfos[i].boundingRect();
//        rectangle(frame,rect.tl(),rect.br(),Scalar(255));
//    }

//    for(int i=0;i<lightInfos.size();i++){
//        auto rect=lightInfos[i].boundingRect();
//        rectangle(orignal,rect.tl(),rect.br(),Scalar(255));
//    }

    for(long unsigned int i=0;i<lightInfos.size();i++)
    {
        for(long unsigned int j=i+1;j<lightInfos.size();j++)
        {
            Light left(lightInfos[i]);
            Light right(lightInfos[j]);
            for(int i=0;i<4;i++)
            {
                circle(frame,left.points[i],3,Scalar(255),-1);
            }
            for(int i=0;i<4;i++)
            {
                circle(frame,right.points[i],3,Scalar(255),-1);
            }

            for(int i=0;i<4;i++)
            {
                circle(orignal,left.points[i],3,Scalar(0,255,0),-1);
            }
            for(int i=0;i<4;i++)
            {
                circle(orignal,right.points[i],3,Scalar(0,255,0),-1);
            }

            Armor aromor(left,right);
//            cout<<"1111111"<<endl;
//            cout<<"aromor.HcomH_LandR       " <<aromor.HcomH_LandR<<endl;
//            cout<<"aromor.WcomH_LandR       " <<aromor.WcomH_LandR<<endl;
//            cout<<"aromor.parallelism       " <<aromor.parallelism<<endl;
//            cout<<"aromor.angle_LandR       " <<aromor.angle_LandR<<endl;

            //边缘装甲板不识别
            
            //筛选
            if(aromor.HcomH_LandR>MAXHCOMHLANDR)
                continue;
            if(aromor.WcomH_LandR>MAXWCOMHLANDR)
               continue;
            if(aromor.WcomH_LandR<MINWCOMHLANDR)
               continue;
            if(aromor.parallelism>MAXPARALLELISM)
               continue;
            if(aromor.angle_LandR>MAXANGLELANDR)
               continue;
            //增强匹配条件筛除干扰装甲板
            //上边缘装甲板舍弃
            aromor.contours_points.clear();
            aromor.contours_points.push_back(left.points[0]);
            aromor.contours_points.push_back(right.points[1]);
            aromor.contours_points.push_back(right.points[3]);
            aromor.contours_points.push_back(left.points[2]);

            bool flag=false;
            for(int i=0;i<(int)aromor.contours_points.size();i++)
            {
                if(aromor.contours_points[i].y<5)
                {
                    flag=true;
                    break;
                }
            }
            if(flag)continue;
            //装甲板外四点   左下逆时针
            aromor.out_points.clear();
            aromor.out_points.push_back(left.points[0]);
            aromor.out_points.push_back(right.points[1]);
            aromor.out_points.push_back(right.points[3]);
            aromor.out_points.push_back(left.points[2]);


            //装甲板矩形
            aromor.armor_rect=minAreaRect(aromor.out_points);
            //筛选装甲板------装甲板的一些属性
            aromor.armor_height=get_ous_distance(aromor.out_points[0],aromor.out_points[3]);
            aromor.armor_width=get_ous_distance(aromor.out_points[0],aromor.out_points[1]);
            double armor_hcomw=aromor.armor_height/aromor.armor_width;
            aromor.armor_rect_area=aromor.armor_rect.size.area();
//            cout<<aromor.armor_rect_area<<"   ";
            if(aromor.armor_rect_area<MINARMORRECTAREA)
                flag=true;

            if(armor_hcomw<1.0)
            {
                armor_hcomw=1.0/armor_hcomw;
            }
            aromor.armor_HcomW=armor_hcomw;
            if(aromor.armor_HcomW>MAXARMORHCOMW&&aromor.out_points[0].y<frame.rows-5)     //宽比高过大并且不是图像边缘
                flag=true;
//            cout<<aromor.armor_HcomW<<"   ";

            if(aromor.armor_rect.size.width<aromor.armor_rect.size.height)
             aromor.armor_rect.angle=180.0 - aromor.armor_rect.angle;

//            cout<<aromor.armor_rect_angle<<"    ";
            if((aromor.armor_rect_angle>=MINARMORANGLE&&aromor.armor_rect_angle<=MAXARMORANGLE))
                flag=true;

//            cout<<aromor.armor_rect_angle<<"    ";

            //if(flag)continue;
//            cout<<aromor.armor_rect_area<<"   ";
//             cout<<aromor.armor_HcomW<<"   ";
            this->armor_filter.push_back(aromor);
//          this->matchLight_deeper();

          }
       }
      matchLight_deeper();
}


void Detect::detect_steps(Mat &frame,Mat & orignal,int color)
{
    int flag_light =0;
    this->preDeal(frame,color);

    //二、找轮廓并处理
    this->contoursDeal(frame);

    //三、通过矩形筛选出灯条
    if(!this->lightInfos.empty())
    {
        flag_light=1;
    }
//            else {
//                cout<<"No contours"<<" ";
//            }

    //四、对灯条匹配筛选   框出目标装甲板
    if(flag_light==1)
    {
        this->matchLight(this->lightInfos,frame,orignal);
        //绘制筛选出的装甲板
        for(int j=0;j<(int)this->armor_filter.size();j++)
        {
        armor_filter[j].draw_armor(orignal,frame);
        }

        //pnp
        for(int j=0;j<(int)this->armor_filter.size();j++)
        {
            PnpDetect pnp;
            pnp.pnp_detect(orignal,armor_filter[j]);
        }

        this->lightInfos.clear();
        this->armor_filter.clear();
    }
}


