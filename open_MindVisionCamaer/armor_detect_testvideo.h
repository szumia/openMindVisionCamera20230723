#ifndef ARMOR_DETECT_TESTVIDEO_H
#define ARMOR_DETECT_TESTVIDEO_H

#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<string>
using namespace std;
using namespace cv;

namespace armor_detect_testvideo {
    void callback(int pos, void*);
    class Test_armor_detect_video
    {
    public:
        std::string setVideo();
        int getColor();
        void test_armor_detect_video();
        void showVideoTrackbar();
        void showVideo();
        Test_armor_detect_video(){};
    };

}


#endif // ARMOR_DETECT_TESTVIDEO_H
