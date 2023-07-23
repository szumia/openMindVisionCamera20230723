#include "armor_detect_testvideo.h"
#include "armor_detect.h"
using Test_armor_detect_video=armor_detect_testvideo::Test_armor_detect_video;

cv::VideoCapture g_cap;
int g_slider_position=0;         //存滑动条位置
int g_run =1;                    //为正 停止   为负 连续播放     为1 单步模式
int g_dontset = 0;               //避免调整进度条时进单步模式
//回调函数，用于createTrackbar
void armor_detect_testvideo::callback(int pos, void*)
{
    g_cap.set(cv::CAP_PROP_POS_FRAMES,pos);
    if(!g_dontset)
    {
        g_run=1;    //单步模式
    }
    g_dontset=0;
}

string Test_armor_detect_video::setVideo()
{
    cout<<"请选择你的测试视频：  【1】 【2】 【3】 【4】 【5】"<<endl;
    int choice;
    cin>>choice;
    string path;
    if(choice==1)
        path="1.avi";
    else if(choice==2)
        path="2.avi";
    else if(choice==3)
        path="3.avi";
    else if(choice==4)
        path="4.avi";
    else if(choice==5)
        path="5.avi";
    else
    {
        cout<<"Input error"<<endl;
        exit(0);
    }
    return path;
}

int Test_armor_detect_video::getColor()
{
    int color;
    cout<<"请输入你需要识别的颜色   【1】蓝色 【2】红色"<<endl;
    int num;
    cin>>num;
    if(num==1)color=1;
    else if(num==2)color=2;
    else
    {
        cout<<"Input error"<<endl;
        return -1;
    }
    return color;
}

//void Test_armor_detect_video::read_video()
//{

//}

//用滑动条
void Test_armor_detect_video::showVideoTrackbar()
{
    string path = this->setVideo();
    cout<<"文件"<<path<<endl;
    g_cap.open(path);
    namedWindow("Test");
    int frames = (int)g_cap.get(CAP_PROP_FRAME_COUNT);
    cout<<"总帧数"<<frames<<endl;
    createTrackbar("Position","Test",NULL,frames,callback);

    //待识别的颜色
    int color = this->getColor();
    if(color==-1)
    {
        cout<<"input error"<<endl;
        return;
    }
    Mat frame;
    cout<<"请选择你想要的播放模式   【s】单步模式   【r】连续模式  【Esc】退出"<<endl;
    for(;;)
    {                                     //标记是否找到装甲板
        if(g_run!=0)
        {
            g_cap>>frame;
//            imshow("313",frame);
//            waitKey(0);
            Mat orignal = frame;
            ////下面我们对frame进行处理，就可以得到不同的视频////
            armor_detect::Detect detect;
            detect.detect_steps(frame,orignal,color);


            if(frame.empty())break;
            int current_pos = (int)g_cap.get(CAP_PROP_POS_FRAMES);  //当前帧数
            g_dontset= 1;

            setTrackbarPos("Position","Test",current_pos);
            //orignal用于输出原图像看效果
            imshow("orignal",orignal);
            imshow("Result",frame);
            g_run -= 1;
        }


        char c = (char)waitKey(10);
        if(c=='s')
        {
            g_run=1;   //g_run等于0跳出循环
            cout<<"单步模式"<<endl;
        }
        if(c=='r')
        {
            g_run = -1;
            cout<<"连续播放模式"<<endl;
        }
        if(c==27)    //按下‘Esc’退出
            break;
    }
}


void Test_armor_detect_video::showVideo()
{
    string path = this->setVideo();
    cout<<path<<endl;
    g_cap.open(path);
    cout<<111111<<endl;
    int frames = (int)g_cap.get(CAP_PROP_FRAME_COUNT);
    cout<<frames<<endl;
    cout<<222222<<endl;

    //待识别的颜色
    int color = this->getColor();
    if(color==-1)
    {
        cout<<"input error"<<endl;
        return;
    }



}




//void Test_armor_detect_video::test_armor_detect_video()
//{

//}
