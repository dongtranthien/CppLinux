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

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int func(int *p)
{
    return (sizeof(p)/sizeof(*p));
}

void TcpRunning(){
  bool isClientConnect = false;

  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};

  #ifdef DEBUG
    printf("\nTcpRunning...");
  #endif

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
      perror("socket failed");
      exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port 8080
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                &opt, sizeof(opt)))
  {
      perror("setsockopt");
      exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons( PORT );

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr *)&address, 
                                sizeof(address))<0)
  {
      perror("bind failed");
      exit(EXIT_FAILURE);
  }
  if (listen(server_fd, 3) < 0)
  {
    printf("\nlisten...");
      perror("listen");
      exit(EXIT_FAILURE);
  }

	while(true){
    if(!isClientConnect){
      if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
      {
        perror("accept");
        exit(EXIT_FAILURE);
      }
      isClientConnect = true;
    }
    else{
      valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );
    }
  }
}

void RosRunning(){
  //ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("map", 5000, mapStore);

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NodeReceiveManualControlTcp");

  std::thread t1(TcpRunning);
  std::thread t2(RosRunning);
  t1.join();
  t2.join();

  return 0;
}