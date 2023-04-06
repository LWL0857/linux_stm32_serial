
#include "ros/ros.h"
#include "std_msgs/String.h" //use data struct of std_msgs/String
#include "mbot_linux_serial.h"
#include "geometry_msgs/PoseStamped.h"
// test send value
double testSend1 = 5555.0;
double testSend2 = 2222.0;
unsigned char testSend3 = 0x07;

// receive mocap value,and reserve pose and orientaion in position and orierntation
double positionX, positionY, positionZ;
double orientationX, orientationY, orientationZ, orientationW;

void serial_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    // pass  msg (mocap publish topic "pose") which has member position and orientation to global position,global orientation
    positionX = msg->pose.position.x;
    positionY = msg->pose.position.y;
    positionZ = msg->pose.position.z;

    orientationX = msg->pose.orientation.x;
    orientationY = msg->pose.orientation.y;
    orientationZ = msg->pose.orientation.z;
    orientationW = msg->pose.orientation.w;

    //ROS_INFO("I heard the pose from mocap");
    //ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    //ROS_INFO("the time we get the pose is %f", msg->header.stamp.sec + 1e-9 * msg->header.stamp.nsec);
    std::cout << "\n \n"
              << std::endl; // add two more blank row so that we can see the message more clearly
}

int main(int agrc, char **argv)
{
    ros::init(agrc, argv, "public_node");
    ros::NodeHandle nh;
    // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数serial_callback
    ros::Subscriber sub = nh.subscribe("vrpn_client_node/body3/pose", 1000, serial_callback);

    ros::Rate loop_rate(10);

    // 串口初始化
    // serialInit();

    while (ros::ok())
    {
        ros::spinOnce();
        // 向STM32端发送接收到的动捕数据，pose和orientation

        write_pose_and_orientation(positionX, positionY, positionZ,
                                   orientationX, orientationY, orientationZ, orientationW);
        // 从STM32接收数据，输入参数依次转化为小车的线速度、角速度、航向角（角度）、预留控制位
        // readSpeed(testRece1,testRece2,testRece3,testRece4);
        // 打印数据
        // ROS_INFO("%f,%f,%f,%d\n",testRece1,testRece2,testRece3,testRece4);

        loop_rate.sleep();
    }
    return 0;
}
