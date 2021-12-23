#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <iterator>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string> 
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[] are laser readings
  float arraySize = scan->ranges.size();
  std::cout<<std::to_string(arraySize) + "\n";
  std::cout<<std::to_string(scan->angle_min) + "-" + std::to_string(scan->angle_max) + "-" + std::to_string(scan->angle_increment) + "-" + std::to_string(scan->range_min) + "-" + std::to_string(scan->range_max) + "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SubscribeTopicScan");

  ros::NodeHandle nh;
  ros::Subscriber scanSub;

  scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",1000, processLaserScan);
  
  ros::spin();
}
