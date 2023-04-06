#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

extern void serialInit();
extern void write_pose_and_orientation(double positionX, double positionY,double  positionZ,
                 double orientationX, double orientationY,double orientationZ,double orientationW);
extern bool read_pose_and_orientation(double &positionX_rec, double &positionY_rec, double &positionZ_rec,
                               double &orientationX_rec, double &orientationY_rec, double &orientationZ_rec, double &orientationW_rec);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
