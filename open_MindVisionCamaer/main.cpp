#include <iostream>
#include "armor_detect_testvideo.h"
#include "Mindvision.h"


using namespace std;

int main()
{
//    armor_detect_testvideo::Test_armor_detect_video test_video;
//    test_video.showVideoTrackbar();

    MindVision::Mindvision camera1;
    //拍照并处理
    //camera1.camera_deal_img();
    //只拍照
    //camera1.get_photos_only();
    //自动拍照
    camera1.get_image_automatic();
    return 0;
}
