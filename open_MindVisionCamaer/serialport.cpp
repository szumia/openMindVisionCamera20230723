#include "serialport.h"

void VisionData::reset()
{
    pitch_angle.f = 0;
    yaw_angle.f = 0;
    dis.f = 0;
    drop_frame = 0;
    isfindDafu = 0;
    isFindTarget = 0;
    nearFace = 0;
    anti_top = 0;
    anti_top_change_armor =0;
}

//发给电控的数据不能为nan等数据，否则会疯车
void VisionData::make_data_safe()
{
    if(pitch_angle.f > 10000 || pitch_angle.f < -10000)
    {
        cout << "pitch越界" <<pitch_angle.f <<endl;
        pitch_angle.f = 0;
    }
    if(yaw_angle.f > 10000 || yaw_angle.f < -10000)
    {
        cout << "yaw越界" << yaw_angle.f<<endl;
        yaw_angle.f = 0;
    }
    if(dis.f > 100000 || dis.f < 0)
    {
        cout <<"dis越界" << dis.f <<endl;
        dis.f = 0;
    }
    if(drop_frame < 0 || drop_frame > 1)
    {
        cout << "drop_frame越界" <<drop_frame <<endl;
        drop_frame < 0;
    }
    if(isfindDafu < 0 || isfindDafu > 1)
    {
        cout << "isfindDafu越界" <<isfindDafu <<endl;
        isfindDafu = 0;
    }
    if(isFindTarget < 0 || isFindTarget > 1)
    {
        cout << "isFindTarget越界" <<isFindTarget <<endl;
        isFindTarget = 0;
    }
    if(anti_top < 0 || anti_top > 1)
    {
        cout << "anti_top越界" <<anti_top <<endl;
        anti_top = 0;
    }
    if(anti_top_change_armor < 0 || anti_top_change_armor > 1)
    {
        cout << "anti_top_change_armor越界" <<anti_top_change_armor <<endl;
        anti_top_change_armor = 0;
    }



}

/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率   每秒传送的二进制位数  衡量数据快慢 相同速率才可解码 
 *@param  databits 类型  int  数据位   取值 为 7 或者8
 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S    无,偶，奇，0 
 *@param  portchar 类型  char* 串口路径
 */
SerialPort::SerialPort()
{
    fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);   //默认打开ttyUSB0
    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}

SerialPort::SerialPort(char *portpath)
{

    fd = open(portpath, O_RDWR | O_NOCTTY | O_NDELAY);
    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
	parity = 'N';
}


// 当串口断了，需要使用该函数重新连接上串口
bool SerialPort::reinitSerialPort(char *portpath)
{
    fd = open(portpath, O_RDWR | O_NOCTTY | O_NDELAY);

    speed = BAUDRATE;
    databits = 8;
    stopbits = 1;
    parity = 'N';
    if (fd == -1)
    {
        perror(UART_DEVICE);
        return false;
    }

    std::cout << "Opening..." << std::endl;
    set_Brate();

    if (set_Bit() == FALSE)
    {
        printf("Set Parity Error\n");
        exit(0);
    }

    printf("Open successed\n");
    return true;
}


union charTofloat
{
    char value[4];
    float fvalue;
};
// 接收数据----电控发的 根据Mode执行不同功能 
bool SerialPort::get_Mode(int &mode, int &shoot_speed, int &my_color,
                          double &gyro_yaw, double &gyro_pitch, int &anti_flag)
{
    //该函数同样需要修改长度，放在宏定义 RDATA_LENGTH
    int bytes;
    int result = ioctl(fd, FIONREAD, &bytes);//该函数用来查看输入缓冲区的总字节数，其中bytes为输入缓冲区的字节数
    if (result == -1)
    {
        printf("ioctl: %s\n", strerror(errno));
        return false;
    }
//    cout<<"输入缓冲区字节数： "<<bytes<<endl;
    if (bytes == 0)
    {
//        cout << "缓冲区为空！" << endl;
        return true;
    }
    /*
     * 下面这段代码的意思是：缓冲区有多少字节我们就读多少字节，这个过程也清空了缓冲区；
     * 然后我们提取出最后一组数据，也就是最新数据。
     */
    bytes = read(fd, rdata, bytes);//read函数将缓冲区的数据读到rdata数组
    int FirstIndex = -1;			//帧头 
    int flag = 0;
    //找到最初一个帧头，把上一帧的残留数据舍去
    for(int i = 0; i < bytes; i++)
    {
        if( rdata[i] == 0xA5 && FirstIndex == -1&& Verify_CRC8_Check_Sum(&rdata[i], 3) && Verify_CRC16_Check_Sum(&rdata[i],RDATA_LENGTH))
        {
            FirstIndex = i;
            flag = 1;
        }
        if(flag)					//找到帧头，退出 
        {
            break;
        }
    }
    int max_mul = (bytes - FirstIndex) / RDATA_LENGTH;					//帧数
//    cout << "FirstIndex: " << FirstIndex <<endl;
//    cout << "bytes: " << bytes << endl;
//    cout << "max_mul: " << max_mul <<endl;

    /*
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH的		意思是最后一组数据的第一位的索引
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH          第0位：帧头
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH + 1      第1位：和电控商量，可以当正常数据接受
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH + 2      第2位：CRC8校验值，未写出，在Verify_CRC8_Check_Sum函数加上，不用修改
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH + 3-13   第3-13位：接受数据
     * FirstIndex + (max_mul - 1)*RDATA_LENGTH + 14-15  第14-15位：CRC16校验的两个字节，在这里我们没有校验，但是要预留这两个字节数
     */

    if(FirstIndex != -1)
    {
        if(rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH] == 0xA5  && Verify_CRC8_Check_Sum(&rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH], 3) )
        {
           /*
            * mode、shoot_speed、my_color、anti_flag 为整型数据
            * gyro_yaw、gyro_pitch、为浮点型数据, 自行百度union结构体是怎么进行数据转换的：内存共享
            */
           mode  = (int)rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH+1];
           shoot_speed  = (int)rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH +3];
           my_color  = (int)rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH+ 4];

           charTofloat cf;
           for(int i = 0; i < 4; i++)
           {
               cf.value[i] = rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH+5+i];
           }
           gyro_yaw = -(double)cf.fvalue;

           for(int i = 0; i < 4; i++)
           {
               cf.value[i] = rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH+9+i];
           }
           gyro_pitch = -(double)cf.fvalue;

           anti_flag = (int)rdata[FirstIndex + (max_mul - 1)*RDATA_LENGTH +13];
        }
        else
        {
            cout<<"CRC8 Check False!!!"<<endl;
        }
    }
    return true;
}
/**
 * 电控信息流       0xA5 mode.i CRC8 shoot_speed.i my_color.i yaw.f pitch.f anti_flag.i CRC16      
 * 					 0	  1       2       3            4       5-8    9-12     13       14-15                  	
 **/


/**
 *@brief   初始化串口
 */
bool SerialPort::initSerialPort()
{

	if (fd == -1)
	{
        perror(UART_DEVICE);
        return false;
    }

	std::cout << "Opening..." << std::endl;
    set_Brate();

	if (set_Bit() == FALSE)
	{
        printf("Set Parity Error\n");
		exit(0);
    }

//    set_disp_mode(0);
    printf("Open successed\n");
    return true;
}

/**
 *@brief   设置波特率
 */
void SerialPort::set_Brate()
{
    int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					  };
    int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					 };
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);


	for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if (speed == name_arr[i])
		{
            tcflush(fd, TCIOFLUSH);                 //清空缓冲区的内容
            cfsetispeed(&Opt, speed_arr[i]);        //设置接受和发送的波特率
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);  //使设置立即生效

			if (status != 0)
			{
                perror("tcsetattr fd1");
                return;
            }

            tcflush(fd, TCIOFLUSH);

        }
    }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 */
int SerialPort::set_Bit()
{
    struct termios termios_p;

	if (tcgetattr(fd, &termios_p)  !=  0)
	{
        perror("SetupSerial 1");
		return (FALSE);
    }

    termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
	termios_p.c_cflag &= ~CSIZE;//设置数据位数

    switch (databits)
    {
	case 7:
		termios_p.c_cflag |= CS7;
		break;

	case 8:
		termios_p.c_cflag |= CS8;
		break;

	default:
		fprintf(stderr, "Unsupported data size\n");
		return (FALSE);
    }

	//设置奇偶校验位double
    switch (parity)
    {
	case 'n':
	case 'N':
		termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
		termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;

	case 'o':
	case 'O':
		termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
		break;

	case 'e':
	case 'E':
		termios_p.c_cflag |= PARENB;     /* Enable parity */
		termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
		termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
		break;

	case 'S':
	case 's':  /*as no parity*/
		termios_p.c_cflag &= ~PARENB;
		termios_p.c_cflag &= ~CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported parity\n");
		return (FALSE);

    }

    /* 设置停止位*/
    switch (stopbits)
    {
	case 1:
		termios_p.c_cflag &= ~CSTOPB;
		break;

	case 2:
		termios_p.c_cflag |= CSTOPB;
		break;

	default:
		fprintf(stderr, "Unsupported stop bits\n");
		return (FALSE);

    }

    /* Set input parity option */
    if (parity != 'n')
        termios_p.c_iflag |= INPCK;

	tcflush(fd, TCIFLUSH); //清除输入缓存区
    termios_p.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    termios_p.c_cc[VMIN] = 0;  //最小接收字符
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
	termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios_p.c_iflag &= ~(ICRNL | IGNCR);
    termios_p.c_oflag &= ~OPOST;   /*Output禁用输出处理*/

	if (tcsetattr(fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
    {
        perror("SetupSerial 3");
        return (FALSE);
    }

    return (TRUE);
}


/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void SerialPort::TransformData(const VisionData &data)
{
    //怎么修改发送数据：按下面模版更改数据，特别留意最后一个函数输入的长度，怎么计算 1个帧头+ 1（整型数据） + 1个CRC8校验值 + 4 * 浮点数据的个数 + 1 *整形数据个数 + 2个CRC16校验值
    Tdata[0] = 0xA5;
    Tdata[1] = data._mode;
    Append_CRC8_Check_Sum(Tdata, 3);//前三位有CRC8校验，第一位为帧头，第二位可以当正常数据发，第三位为CRC校验值

    //浮点数据用4个字节； 整型数据用1个字节
    //浮点数据: yaw(x); pit(y); dis(z)
    Tdata[3] = data.pitch_angle.c[0];
    Tdata[4] = data.pitch_angle.c[1];
    Tdata[5] = data.pitch_angle.c[2];
    Tdata[6] = data.pitch_angle.c[3];

    Tdata[7] = data.yaw_angle.c[0];
    Tdata[8] = data.yaw_angle.c[1];
    Tdata[9] = data.yaw_angle.c[2];
    Tdata[10] = data.yaw_angle.c[3];

    Tdata[11] = data.dis.c[0];
    Tdata[12] = data.dis.c[1];
    Tdata[13] = data.dis.c[2];
    Tdata[14] = data.dis.c[3];
    //整型数据: 一些标志位
    Tdata[15] = data.drop_frame;
	Tdata[16] = data.isFindTarget;
    Tdata[17] = data.isfindDafu;
    Tdata[18] = data.nearFace;
    Tdata[19] = data.anti_top;
    Tdata[20] = data.anti_top_change_armor;

    Append_CRC16_Check_Sum(Tdata, 23);//后两位为CRC校验值，这里是CRC16也就是检验两个字节，留意23怎么计算的
}
/**
 * 视觉信息流       0xA5 mode.i CRC8  pitch.f  yaw.f   dis.f   biaozhi.i   CRC16      
 * 					 0	  1       2    3-6      7-10   11-14     15-20     21-22 	
 **/






//封装数据后还需要发送数据
void SerialPort::send()
{
    result = write(fd, Tdata, 23);
//    cout<<result<<endl;
}

//关闭通讯协议接口
void SerialPort::closePort()
{
    close(fd);
}
