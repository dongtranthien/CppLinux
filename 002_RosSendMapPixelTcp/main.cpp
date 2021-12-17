#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
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
#define PORT_WITH_APP 2002
#define PORT_WITH_MAIN_PROCESS 2005

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int func(int *p)
{
    return (sizeof(p)/sizeof(*p));
}

int a[7] = {1,2,3,4,5,6,7};
unsigned char result[10000000] = {0};
void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->info.width);
  ROS_INFO("I heard: [%d]",msg->info.height);
  
  using namespace std;
  int arr[] = {2, 7, 1, 111};
  auto array_length = end(msg->data) - begin(msg->data);
  
  ROS_INFO("array_length: [%d]",array_length);
  unsigned long index;
  unsigned long max = 0;
  for(index = 0; index < array_length; index++){
    if((msg->data[index] > max)&&(msg->data[index] != (-1))){
      max = msg->data[index];
    }
  }

  unsigned char temp;
  unsigned long indexResult = 0;
  for(index = 0; index < (array_length - 8); index+=8){
    temp = 0;
    if(msg->data[index] > 30){
      temp |= 0x01;
    }
    if(msg->data[index + 1] > 30){
      temp |= 0x02;
    }
    if(msg->data[index + 2] > 30){
      temp |= 0x04;
    }
    if(msg->data[index + 3] > 30){
      temp |= 0x08;
    }
    if(msg->data[index + 4] > 30){
      temp |= 0x10;
    }
    if(msg->data[index + 5] > 30){
      temp |= 0x20;
    }
    if(msg->data[index + 6] > 30){
      temp |= 0x40;
    }
    if(msg->data[index + 7] > 30){
      temp |= 0x80;
    }

    result[indexResult] = temp;
    indexResult++;
  }

  ROS_INFO("max: [%d]",max);
}

nav_msgs::OccupancyGrid mapData;
void mapStore(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_INFO("msg->info.width: [%d]",msg->info.width);
  ROS_INFO("msg->info.height: [%d]",msg->info.height);
  ROS_INFO("msg->info.resolution: [%f]",msg->info.resolution);
  ROS_INFO("msg->info.origin.position.x: [%f]",msg->info.origin.position.x);
  ROS_INFO("msg->info.origin.position.y: [%f]",msg->info.origin.position.y);
  ROS_INFO("msg->info.origin.position.z: [%f]",msg->info.origin.position.z);
  ROS_INFO("msg->info.origin.orientation.x: [%f]",msg->info.origin.orientation.x);
  ROS_INFO("msg->info.origin.orientation.y: [%f]",msg->info.origin.orientation.y);
  ROS_INFO("msg->info.origin.orientation.z: [%f]",msg->info.origin.orientation.z);
  ROS_INFO("msg->info.origin.orientation.w: [%f]",msg->info.origin.orientation.w);
  
  using namespace std;
  int arr[] = {2, 7, 1, 111};
  auto array_length = end(msg->data) - begin(msg->data);
  
  ROS_INFO("array_length: [%d]",array_length);
  unsigned long index;
  unsigned long max = 0;
  for(index = 0; index < array_length; index++){
    
  }
  mapData = *msg;
  //ROS_INFO("I 1: [%d]",msg->info.width);
  //ROS_INFO("I 2: [%d]",msg->info.height);
}

void TcpRunningWithApp(){
  bool isClientConnect = false;

  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};
  char *hello = "Hello from server";

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
  address.sin_port = htons( PORT_WITH_APP );

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
      if(valread == 0){
        isClientConnect = false;
      }
      else if(strncmp("{Request}", buffer, 9) == 0){
        printf("Received\n");

        unsigned long lengthData = mapData.info.width*mapData.info.height;
        unsigned long indexCurrent = 0;

        if(send(new_socket, "{", 1, 0) < 0){
          isClientConnect = false;
        }

        std::string widthStr = std::to_string(mapData.info.width);
        std::string heightStr = std::to_string(mapData.info.height);
        std::string resolutionStr = std::to_string(mapData.info.resolution);
        std::string originPosXStr = std::to_string(mapData.info.origin.position.x);
        std::string originPosYStr = std::to_string(mapData.info.origin.position.y);
        std::string originPosZStr = std::to_string(mapData.info.origin.position.z);
        std::string originOriXStr = std::to_string(mapData.info.origin.orientation.x);
        std::string originOriYStr = std::to_string(mapData.info.origin.orientation.y);
        std::string originOriZStr = std::to_string(mapData.info.origin.orientation.z);
        std::string originOriWStr = std::to_string(mapData.info.origin.orientation.w);
        std::string headerStr = widthStr + "," + 
                                     heightStr + "," + 
                                     resolutionStr + "," + 
                                     originPosXStr + "," + 
                                     originPosYStr + "," + 
                                     originPosZStr + "," + 
                                     originOriXStr + "," + 
                                     originOriYStr + "," + 
                                     originOriZStr + "," + 
                                     originOriWStr + ",";
        if(isClientConnect&&(send(new_socket, headerStr.c_str(), headerStr.length(), 0) < 0)){
          isClientConnect = false;
        }

        printf("Send data: %d\n", lengthData);
        while(isClientConnect&&(lengthData > indexCurrent)){
          unsigned int lengthSend;
          if((lengthData - indexCurrent) >= 8192){
            lengthSend = 8192;
          }
          else{
            lengthSend = (lengthData - indexCurrent);
          }

          

          if(send(new_socket , &mapData.data[indexCurrent], lengthSend, 0) < 0){
            isClientConnect = false;
            printf("aa--------------------------------------------");
            break;
          }

          indexCurrent += lengthSend;
          usleep(2000);
        }
        printf("--------------------------------------------");
        usleep(1000);
        if(send(new_socket, "}", 1, 0) < 0){
          isClientConnect = false;
        }
      }
    }
  }
}
void TcpRunningWithMainProcess(){
  bool isClientConnect = false;

  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};
  char *hello = "Hello from server";

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
  address.sin_port = htons( PORT_WITH_MAIN_PROCESS );

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
      if(strncmp("{Request}", buffer, 9) == 0){
        printf("Received\n");

        unsigned long lengthData = mapData.info.width*mapData.info.height;
        unsigned long indexCurrent = 0;

        if(send(new_socket, "{", 1, 0) < 0){
          isClientConnect = false;
        }

        std::string widthStr = std::to_string(mapData.info.width);
        std::string heightStr = std::to_string(mapData.info.height);
        std::string widthHeightStr = widthStr + "," + heightStr + ",";
        if(isClientConnect&&(send(new_socket, widthHeightStr.c_str(), widthHeightStr.length(), 0) < 0)){
          isClientConnect = false;
        }

        printf("Send data: %d\n", lengthData);
        while(isClientConnect&&(lengthData > indexCurrent)){
          unsigned int lengthSend;
          if((lengthData - indexCurrent) >= 8192){
            lengthSend = 8192;
          }
          else{
            lengthSend = (lengthData - indexCurrent);
          }

          

          if(send(new_socket , &mapData.data[indexCurrent], lengthSend, 0) < 0){
            isClientConnect = false;
            printf("aa--------------------------------------------");
            break;
          }

          indexCurrent += lengthSend;
          usleep(2000);
        }
        printf("--------------------------------------------");
        usleep(1000);
        if(send(new_socket, "}", 1, 0) < 0){
          isClientConnect = false;
        }
      }
    }
  }
}
void RosRunning(){
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 5000, mapStore);

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  std::thread t1(TcpRunningWithApp);
  std::thread t2(RosRunning);
  t1.join();
  t2.join();

  return 0;
}