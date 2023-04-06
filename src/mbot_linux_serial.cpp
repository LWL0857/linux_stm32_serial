#include "mbot_linux_serial.h"


using namespace std;
using namespace boost::asio;
// 串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

// 发送动捕接收的数据给飞控 共用体
union sendData
{
    double d;
    unsigned char data[8];
} leftVelSet,rightVelSet,positionX_set, positionY_set, positionZ_set, orientationX_set, orientationY_set, orientationZ_set, orientationW_set;

// 接收飞控数据给linux 共用体
union receiveData
{
    double d;
    unsigned char data[8];
} positionX_rec, positionY_rec, positionZ_rec, orientationX_rec, orientationY_rec, orientationZ_rec, orientationW_rec;

unsigned char buf[62] = {0}; //
/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
}

/********************************************************
函数功能：将动捕接收的数据，打包发送给下位机
入口参数：pose and orientation
出口参数：
********************************************************/
void write_pose_and_orientation(double positionX, double positionY, double positionZ,
                                double orientationX, double orientationY, double orientationZ, double orientationW)
{
    
    int i, length = 0;
    ROS_INFO("the third position x is %f ", positionX);

    positionX_set.d = positionX; // mm/s
    positionY_set.d = positionY;
    positionZ_set.d = positionZ;
    orientationX_set.d = orientationX;
    orientationY_set.d = orientationY;
    orientationZ_set.d = orientationZ;
    orientationW_set.d =orientationW;
    ROS_INFO("the ***** position x is %f ", positionX_set.d);
    // 设置消息头
    for (i = 0; i < 2; i++)
        buf[i] = header[i]; // buf[0]  buf[1]

    // 设置机器人左右轮速度
    length = 56;
    buf[2] = length; // buf[2]

    memcpy(&buf[3], &positionX_set.data, 8);
    memcpy(&buf[11], &positionY_set.data, 8);
    memcpy(&buf[19], &positionZ_set.data, 8);
    memcpy(&buf[27], &orientationX_set.data, 8);
    memcpy(&buf[35], &orientationY_set.data, 8);
    memcpy(&buf[43], &orientationZ_set.data, 8);
    memcpy(&buf[51], &orientationW_set.data, 8);
 /*
    for (i = 0; i < 7; i++)
    {
       buf[i + 3] =positionX_set.data[i];  // buf[3]-buf[10]
       //ROS_INFO("the ***** position x is %s ", positionX_set.data[i]);
       buf[i + 11] =positionX_set.data[i]; // buf[11]-buf[18].
       buf[i + 19] =positionX_set.data[i];// buf[19]-buf[26]
       buf[i + 27] =orientationX_set.data[i];// buf[17]-buf[34]
       buf[i + 35] =orientationX_set.data[i];
       buf[i + 43] =orientationX_set.data[i];
       buf[i + 51] =orientationX_set.data[i];// buf[51]-buf[58]
    }*/
    // 预留控制指令
    //buf[3 + length - 1] = ctrlFlag; // buf[52]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length); // buf[59]
    buf[3 + length + 1] = ender[0];             // buf[60]
    buf[3 + length + 2] = ender[1];             // buf[61]
    
    positionX_set.data[0] = buf[3];
    positionX_set.data[1] = buf[4];
    positionX_set.data[2] = buf[5];
    positionX_set.data[3] = buf[6];
    positionX_set.data[4] = buf[7];
    positionX_set.data[5] = buf[8];
    positionX_set.data[6] = buf[9];
    positionX_set.data[7] = buf[10];
    for (int i = 0; i < 8; i++)
    {

        cout << i << "***** positionX_set.d is " << positionX_set.data[i] << endl;
    }
   
    cout << " positionX_set.d is " << positioSUOnX_set.d << endl;
    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}
/********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
********************************************************/
/*bool readSpeed(double &Left_v, double &Right_v, double &Angle, unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[150] = {0};
    //=========================================================
    // 此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n", err);
        copy(istream_iterator<unsigned char>(istream(&response) >> noskipws),
             istream_iterator<unsigned char>(),
             buf);
    }
    catch (boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    }
    //=========================================================

    // 检查信息头
    if (buf[0] != header[0] || buf[1] != header[1]) // buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = buf[2]; // buf[2]

    // 检查信息校验值
    checkSum = getCrc8(buf, 3 + length); // buf[10] 计算得出
    if (checkSum != buf[3 + length])     // buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }

    // 读取速度值
    for (i = 0; i < 2; i++)
    {
        leftVelNow.data[i] = buf[i + 3];  // buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; // buf[5] buf[6]
        angleNow.data[i] = buf[i + 7];    // buf[7] buf[8]
    }

    // 读取控制标志位
    ctrlFlag = buf[9];

    Left_v = leftVelNow.d;
    Right_v = rightVelNow.d;
    Angle = angleNow.d;

    return true;
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--)
    {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}
