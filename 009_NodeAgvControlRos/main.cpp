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
uint32_t velocityLeft = 0, velocityRight = 0;

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
  controlDat = std::stoi(msg->data);
}

void CalculateVelocityControlDriver(){
  if((controlDat == 0)||(controlDat == 9)){
    velocityLeft = 0;
    velocityRight = 0;
  }
  else{
    velocityLeft = 1;
    velocityRight = 1;
  }
}

void RosRunning(){
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("manualControl", 5000, ReceiveDataControlToSendDriver);

  ros::NodeHandle nPub;
  ros::Publisher velocityDataPub = nPub.advertise<std_msgs::String>("driverControl", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok()){
    CalculateVelocityControlDriver();

    std_msgs::String msg;
    std::string datSend;
    datSend = std::to_string(velocityLeft) + "," + std::to_string(velocityRight);
    msg.data = datSend;
    velocityDataPub.publish(msg);

    printf("Velocity send to Can Node: %s\n", datSend.c_str());

    ros::spinOnce();

    loop_rate.sleep();
  }
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