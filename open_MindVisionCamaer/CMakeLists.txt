cmake_minimum_required(VERSION 3.24)
project(open_MindVisionCamera)

set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIRS} )



add_executable(open_MindVisionCamera main.cpp armor_detect.cpp armor_detect_testvideo.cpp Mindvision.cpp pnp_detect.cpp serialport.cpp CRC_Check.cpp)
link_directories("/usr/local/lib/")
target_link_libraries( open_MindVisionCamera ${OpenCV_LIBS}  "/usr/lib/libMVSDK.so")