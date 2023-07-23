#ifndef SERIALPORT_H
#define SERIALPORT_H


#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "CRC_Check.h"
using namespace std;

#define TRUE 1
#define FALSE 0


#define RDATA_LENGTH 16  			//电信传视觉的信息流长度 

//串口的相关参数
#define BAUDRATE 115200//波特率
#define UART_DEVICE "/dev/ttyUSB0"//默认的串口名称

//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

//字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

//用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
    float2uchar pitch_angle;//俯仰角
	float2uchar yaw_angle;//偏航角
	float2uchar dis;//目标距离
    int drop_frame;
    int isFindTarget;
    int isfindDafu;
    int nearFace;
    int anti_top;
    int anti_top_change_armor;
    int _mode;

    void reset();
    void make_data_safe();
} VisionData;


class SerialPort
{
private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[4096]; //raw_data
    unsigned char Tdata[30];  //transfrom data
    int drop_cnt;
	void set_Brate();
	int set_Bit();

public:
    int result;
    SerialPort();
    SerialPort(char *);
    bool initSerialPort();
    bool get_Mode(int &mode, int &shoot_speed, int &my_color,
                  double &gyro_yaw, double &gyro_pitch, int &anti_flag);
	void TransformData(const VisionData &data); //主要方案
	void send();
	void closePort();
	void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1
    bool reinitSerialPort(char *portpath);
//  int set_disp_mode(int);
//  void TransformTarPos(const VisionData &data);
};

#endif //SERIALPORT_H
