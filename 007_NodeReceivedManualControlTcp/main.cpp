#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iterator>
#include <iostream>
#include <thread>
#include <pthread.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <string> 
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define DEBUG
#define PORT 2003

int controlDat = 0;
uint32_t counter = 0, counterPre = 0;

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
      //printf("%s\n",buffer );
      //printf("%d\n",valread );

      char bufferCheck[] = "{\"name\":\"manualControl\",\"direct\":";
      int index;
      for(index = 0; index < 33; index++){
        if(buffer[index] != bufferCheck[index]){
          break;
        }
      }

      if(index == 33){
        controlDat = buffer[33] - '0';
      }
      else{
        controlDat = 0;
      }

      counter++;

      if(valread == 0){
        isClientConnect = false;
      }
    }
  }
}

void RosRunning(){
  //ros::NodeHandle n;
  //system("sudo kill -9 `sudo lsof -t -i:2003`");

  //ros::Subscriber sub = n.subscribe("map", 5000, mapStore);
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("manualControl", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok()){
    if(counter != counterPre){
      counterPre = counter;
    }
    else{
      controlDat = 9;
    }

    printf("Control: %d\n", controlDat);

    std_msgs::String msg;
    msg.data = std::to_string(controlDat);
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
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