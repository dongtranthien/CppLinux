#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iterator>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <string> 

#define DEBUG
#define PORT 2003

int controlDat = 0;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int func(int *p)
{
    return (sizeof(p)/sizeof(*p));
}

void ReceiveDataControlToSendDriver(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void RosRunning(){
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("manualControl", 5000, ReceiveDataControlToSendDriver);

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NodeAgvControlRos");

  //std::thread t1(TcpRunning);
  std::thread t2(RosRunning);
  //t1.join();
  t2.join();

  return 0;
}