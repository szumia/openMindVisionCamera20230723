#include "Mindvision.h"
#include "armor_detect.h"

using Mindvision=MindVision::Mindvision;

Mindvision::~Mindvision()
{
    cout<<"Camera Closed!"<<endl;
    CameraUnInit(camera_handle);
        free(g_pRgbBuffer);
}

void Mindvision::init_camera(int tigger_mode)
{
    //初始化sdk
    CameraSdkInit(1);
    int                         status=-1;
    int                         camera_counts=1;
    tSdkCameraDevInfo           tCameraEnumList;

    //枚举 建立设备列表
    status = CameraEnumerateDevice(&tCameraEnumList,&camera_counts);
    if(status!=0)
    {
        cout<<"没有连接相机设备"<<endl;
        return;
    }
    else {
        cout<<"成功找到 "<<camera_counts<<" 台相机设备"<<endl;
    }


    //初始化相机   并绑定句柄
    status=CameraInit(&tCameraEnumList,-1,-1,&camera_handle);
    if(status!= CAMERA_STATUS_SUCCESS)
    {
        cout<<"Init camera fault!"<<endl;
        return;
    }
    else {
        cout<<"Init successfully!"<<endl;
    }

    //获取相机参数
    CameraGetCapability(camera_handle,&tCapability);
    //分配当前帧内存
    g_pRgbBuffer=(unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

     //相机进入工作模式   设置曝光模式
    CameraPlay(camera_handle);
    CameraSetAeState(camera_handle,FALSE);

    //设置硬触发方式
    if(tigger_mode==2)
    {
        CameraSetTriggerMode(camera_handle,2);
        CameraSetExtTrigSignalType(camera_handle,EXT_TRIG_LOW_LEVEL);
    }



    //////////////////////相机参数//////////////////////
    //读取曝光时间
    double expourse = 0;
    double Expourse = 15000;
    CameraSetExposureTime(camera_handle,Expourse);
    CameraGetExposureTime(camera_handle,&expourse);
    cout<<"曝光时间为 "<<expourse<<endl;


    int gama =120;
    status=CameraSetGamma(camera_handle,gama);
    status = CameraGetGamma(camera_handle,&gama);
    cout<<"Gama为 "<<gama<<endl;


    //    读对比度
    int contrast=0;
      //status = CameraSetGamma(camera_handle,2);
      status = CameraGetGamma(camera_handle,&contrast);
    cout<<"对比度为： "<< contrast<<endl;

    //    读增益
    int r=0,g=0,b=0;
    //CameraSetGain(camera_handle,r,g,b);
    CameraGetGain(camera_handle,&r,&g,&b);
    cout<<"增益值为"<<"r:"<<r<<" g:"<<g<<" b:"<<b<<endl;

    //////////////////// 设置分辨率 //////////////////////
    //分辨率信息描述
    tSdkImageResolution pImageResolution = {0};                                    //P40       设置当前的分辨率。
    pImageResolution.iIndex = 0xff;                                                 //表示自定义分辨率（roi）
    pImageResolution.iHeight = 1024;                                                 //相机最终输出的图像的高度
    pImageResolution.iHeightFOV = 1024;                                              //采集视场的高度
    pImageResolution.iVOffsetFOV = 304;                                             //采集视场相对亍 Sensor 最大视角左上角的水平偏移
    pImageResolution.iHOffsetFOV = 0;                                               //采集视场相对亍 Sensor 最大视角左上角的垂直偏移
    pImageResolution.iWidth = 1280;                                                 //相机最终输出的图像的宽度
    pImageResolution.iWidthFOV = 1280;                                              //采集视场的宽度
    status = CameraSetImageResolution(camera_handle, &pImageResolution);                 //设置预览的分辨率。


    // 设置图像处理的输出格式    设置图像处理的输出格式，彩色黑白都支持RGB24位
    if(tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(camera_handle,CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel=3;
        CameraSetIspOutFormat(camera_handle,CAMERA_MEDIA_TYPE_BGR8);
    }
}


void Mindvision::get_image()
{
    int status = -1;
    status = CameraGetImageBuffer(camera_handle,&sFrameInfo,&pybBuffer,1000);
    if(status== CAMERA_STATUS_SUCCESS)
    {
        status = CameraImageProcess(camera_handle,pybBuffer,g_pRgbBuffer,&sFrameInfo);
        if(!(status==CAMERA_STATUS_SUCCESS))
        {
            cout<<"CameraImageProcess Failed"<<endl;
        }
        else
        {
           //获取一帧图像
           src =cv::Mat(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType== CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
           status=CameraReleaseImageBuffer(camera_handle,pybBuffer);
           if(status!=CAMERA_STATUS_SUCCESS)
           {
            cout<<"API: CameraReleaseImageBuffer failed: "<< status <<endl;
           }
        }
    }
    else
    {
          cout<<"API: CameraGetImageBuffer failed: " << status << endl;
          struct tm *newtime;
          char tmpbuf[128];
          time_t test;
          time(&test);
          newtime=localtime(&test);
          strftime(tmpbuf, 128, "%c", newtime);
          cout<<tmpbuf<<"相机异常状态"<<endl;
          exit(-1);
    }
}

// camera_mode=0 默认拍照并处理     1 只拍照
void Mindvision::control_camera(int camera_mode)
{
    int    imageorder=0;
    int    color=1;
    string imagename;
    if(camera_mode==0)
   {
    cout<<"请输入你需要识别的颜色   【1】蓝色 【2】红色"<<endl;
    cin>>color;
   }
 cout<<"操作： 【s/d】切换曝光 【f】调整参数 【】保存图片  【Esc】退出"<<endl;
    while(1)
    {
        //得到图片
        this->get_image();

        //////图像处理-----降噪等///////
        if(camera_mode==0)
        {
            //处理装甲板
            cv::Mat orignal=src;
            armor_detect::Detect detect;
            detect.detect_steps(src,orignal,color);
            cv::imshow("orignal",orignal);
            cv::imshow("Result",src);
        }
        if(camera_mode==1)
        {
            cv::namedWindow("rgb",cv::WINDOW_AUTOSIZE);
            cv::imshow("rgb",src);
        }
        //////////////////////////////
        char key = cv::waitKey(10);
        //低曝光
        if(key=='s')
        {
            double Expourse = 3500;
            CameraSetExposureTime(camera_handle,Expourse);
        }
        //高曝光
        if(key == 'd')
        {
            double expourse = 15000;
            CameraSetExposureTime(camera_handle,expourse);
        }

        //调参数
        if(key=='f')
        {
            cout<<"请输入要调节的参数： 【e】曝光 【g】gama 【r】增益"<<endl;
            char choice;
            cin>>choice;
            if(choice=='e')
            {
                double baoguang;
                cout<<"输入曝光时间"<<endl;
                cin>>baoguang;
                CameraSetExposureTime(camera_handle,baoguang);
                CameraGetExposureTime(camera_handle,&baoguang);
                cout<<"曝光时间为"<<baoguang<<endl;
            }
            if(choice=='g')
            {
                int gama;
                cout<<"输入gamma"<<endl;
                cin>>gama;
                CameraSetGamma(camera_handle,gama);
                CameraGetGamma(camera_handle,&gama);
                cout<<"gama为"<<gama<<endl;
            }
            if(choice=='r')
            {
                int r,g,b;
                cout<<"请输入增益值 r,g,b"<<endl;
                cin>>r>>g>>b;
                CameraSetGain(camera_handle,r,g,b);
                CameraGetGain(camera_handle,&r,&g,&b);
                cout<<"增益值为"<<"r:"<<r<<" g:"<<g<<" b:"<<b<<endl;
            }

        }


        //保存图片
        if(key=='q')
        {
            imagename ="./photos/"+to_string(imageorder++)+".jpg";
            imwrite(imagename,src);
            cout<<imagename<<"图片保存成功"<<endl;
        }


         // 'Esc'  退出
         if(cv::waitKey(1)==27)
         {
             break;
         }
    }
}

void Mindvision::get_image_automatic(int camera_mode)
{
    int tigger_mode=0;
//    cout<<"Please input your tigger_mode: 【0】连续取图  【2】硬触发"<<endl;
//    cin>>tigger_mode;
    //Mindvision camera1;
    this->init_camera(tigger_mode);


    int imageorder = 0;
    int color = 1;
    string imagename;
    int click_down = CLICK_DOWN;
    int intervel = INTERVAL;
    int max_photos_sum=PHOTOSNUM;
    int photos_sum_now=0;
    namedWindow("CountDown",WINDOW_AUTOSIZE);


        while(click_down>0)
        {
            double time=0;
            while(1)
            {
                auto start=std::chrono::high_resolution_clock ::now();
                this->get_image();
                if(camera_mode==0)
                {
                    //处理装甲板
                    cv::Mat orignal=src;
                    armor_detect::Detect detect;
                    detect.detect_steps(src,orignal,color);
                    cv::imshow("orignal",orignal);
                    cv::imshow("Result",src);
                }

                putText(src, to_string(click_down),Point(src.cols/2,src.rows/2),
                        FONT_HERSHEY_SIMPLEX,6,Scalar(0,0,255),5);
                imshow("CountDown",src);

                waitKey(1);      //1ms
                auto end=std::chrono::high_resolution_clock ::now();
                auto duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
                time+=duration.count();
                if(time>=1000)
                {
                    break;
                }
            }
                click_down--;
        }


        while(photos_sum_now<max_photos_sum)
        {
            while(intervel>0)
            {
                double time=0;
                while(1)
                {
                    auto start=std::chrono::high_resolution_clock ::now();
                    this->get_image();
                    if(camera_mode==0)
                    {
                        //处理装甲板
                        cv::Mat orignal=src;
                        armor_detect::Detect detect;
                        detect.detect_steps(src,orignal,color);
                        cv::imshow("orignal",orignal);
                        cv::imshow("Result",src);
                    }

                    putText(src, to_string(imageorder)+".jpg",Point(50,50),
                            FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),3);
//                    putText(src, to_string(intervel),Point(50,50),
//                            FONT_HERSHEY_SIMPLEX,2,Scalar(0,255,0),3);
                    imshow("CountDown",src);

                    waitKey(1);      //1ms
                    auto end=std::chrono::high_resolution_clock ::now();
                    auto duration=std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
                    time+=duration.count();
                    if(time>=RATE)
                    {
                        break;
                    }
                }
                intervel--;
            }

            //保存图片
            imagename ="./photos/"+to_string(imageorder++)+".jpg";
            imwrite(imagename,src);
            cout<<imagename<<"图片保存成功"<<endl;

            photos_sum_now++;
            intervel=INTERVAL;

        }
}

//不处理只拍照
void Mindvision::get_photos_only()
{
    int tigger_mode;
    cout<<"Please input your tigger_mode: 【0】连续取图  【2】硬触发"<<endl;
    cin>>tigger_mode;
    Mindvision camera1;
    camera1.init_camera(tigger_mode);
    camera1.control_camera(1);
}


//相机读图处理一条龙
void Mindvision::camera_deal_img()
{
    int tigger_mode;
    cout<<"Please input your tigger_mode: 【0】连续取图  【2】硬触发"<<endl;
    cin>>tigger_mode;
    Mindvision camera1;
    camera1.init_camera(tigger_mode);
    camera1.control_camera();
}
