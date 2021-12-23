#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

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
#include <arpa/inet.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string> 
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#include <sstream>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define PORT_COMMUNICATE_WITH_APP 2007
#define PORT_COMMUNICATE_WITH_MAIN_PROCESS 2006

typedef enum{
  SERVER_FD_COMMUNICATE_WITH_APP = 0,
  SERVER_FD_COMMUNICATE_TOTAL
}ServerFileDescription;
typedef struct{
  int ServerFd;
  struct sockaddr_in Address;
  bool IsConnect = false;
  int Socket;
}SocketTcpParameter;
SocketTcpParameter socketTcpParameter[SERVER_FD_COMMUNICATE_TOTAL];
bool isNeedData = false;
std::string laserData;
bool isReceivedLaserData = false;
tf::StampedTransform transform;

void RosRunning();
void CommunicateWithApp();
void CommunicateWithMainProcess();
void ProcessLaserScannerData(const sensor_msgs::LaserScan::ConstPtr& scan);
SocketTcpParameter InitTcpSocket(uint16_t port, uint16_t timeoutMs);


void ProcessLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
  //scan->ranges[] are laser readings
  float arraySize = scan->ranges.size();
  std::cout<<std::to_string(arraySize) + "\n";
  std::cout<<std::to_string(scan->angle_min) + "-" + std::to_string(scan->angle_max) + "-" + std::to_string(scan->angle_increment) + "-" + std::to_string(scan->range_min) + "-" + std::to_string(scan->range_max) + "\n";
}

int main(int argc, char **argv){
  ros::init(argc, argv, "AgvRosNodeSendLidarAndPos");

  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP] = InitTcpSocket(PORT_COMMUNICATE_WITH_APP, 0);

  std::thread t1(RosRunning);
  std::thread t2(CommunicateWithApp);
  std::thread t3(CommunicateWithMainProcess);
  //std::thread t4(ThreadStartSendMapNode);
  
  t1.join();
  t2.join();
  t3.join();
  
  ros::spin();

}

void RosRunning(){
  usleep(4000000);
  
  ros::NodeHandle nh;
  ros::Subscriber scanSub;
  scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, ProcessLaserScannerData);

  ros::NodeHandle node;
  tf::TransformListener listener;

  ros::Rate r(10.0);
  while(1){
    ros::spinOnce();
    
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    std::cout<<std::to_string(transform.getOrigin().x()) + "-" + std::to_string(transform.getOrigin().y()) + "-" + std::to_string(transform.getRotation().x()) + "-" + std::to_string(transform.getRotation().y()) + "-" + std::to_string(transform.getRotation().z()) + "-" + std::to_string(transform.getRotation().w()) + "\n";

    r.sleep();
  }
}

void CommunicateWithApp(){
  struct sockaddr_in address_t;
  int addrlen = sizeof(address_t);
  char buffer[1024] = {0};
  int new_socket;

  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].IsConnect = false;
  while(true){
    if(!socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].IsConnect){
      if ((new_socket = accept(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].ServerFd, 
                        (struct sockaddr *)&socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].Address, 
                        (socklen_t*)&addrlen))<0)
      {
        //perror("accept");
        //exit(EXIT_FAILURE);
        
      }
      else{
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].IsConnect = true;
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].Socket = new_socket;

        std::cout<<"CommunicateWithApp Connect..\n";
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].IsConnect = false;
        std::cout<<"CommunicateWithApp Disconnect..\n";
        //while(1);
      }
      else{
        //send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].Socket, laserData.c_str(), laserData.length(), 0);
        if(strncmp("{RequestLaserAndPos}", buffer, 20) == 0){
          if(isReceivedLaserData){
            printf("Send data..\n");
            send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].Socket, laserData.c_str(), laserData.length(), 0);
          }
          else{
            printf("LaserAndPosWaiting..\n");
            send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].Socket, "{LaserAndPosWaiting}", 20, 0);
          }
        }
      }
    }
  }
}

void CommunicateWithMainProcess(){
  bool isServerExist = false;

  int server_fd, new_socket, valread;
  int sock = 0;
  struct sockaddr_in address;
  struct sockaddr_in serv_addr;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};

  #ifdef DEBUG
    printf("CommunicateWithMainProcess - TcpRunning...\n");
  #endif

  // Creating socket file descriptor
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("CommunicateWithMainProcess - socket failed");
    exit(EXIT_FAILURE);
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT_COMMUNICATE_WITH_MAIN_PROCESS);
      
  // Convert IPv4 and IPv6 addresses from text to binary form
  if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
  {
    printf("CommunicateWithMainProcess - Invalid address/ Address not supported \n");
    while(1){
      usleep(100000);
    }
  }
  
  while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) > 0)
  {
    usleep(100000);
  }

	while(true){
    valread = read( sock , buffer, 1024);
    std::cout<<"CommunicateWithMainProcess - Read: " + std::to_string(valread)+"\n";
    printf("%s\n", buffer);
    //usleep(1000);
    if((valread == 0)||(valread == (-1))){
      printf("CommunicateWithMainProcess - Reconnect...\n");
      close(sock);
      if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == 0)
      {
        perror("CommunicateWithMainProcess - socket failed");
        exit(EXIT_FAILURE);
      }
      while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
      {
        usleep(100000);
        //printf("CommunicateWithMainProcess - Reconnect.....\n");
      }
      printf("CommunicateWithMainProcess - Reconnect Ok!\n");
    }
    else{
      if(strncmp(buffer, "RosSendLidarAndPosStart", 23) == 0){
        printf("CommunicateWithMainProcess - Start send lidar and pos");
        isNeedData = true;
        //isMapping = true;
      }
      else if(strncmp(buffer, "RosSendLidarAndPosStop", 22) == 0){
        printf("CommunicateWithMainProcess - Stop send lidar and pos");
        isNeedData = false;
        isReceivedLaserData = false;
        //isMapping = false;
      }
    }
  }
}

void ProcessLaserScannerData(const sensor_msgs::LaserScan::ConstPtr& scan){
  if(!isNeedData) return;
  if(!socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP].IsConnect) return;

  isReceivedLaserData = true;

  tf::Quaternion q(
      transform.getRotation().x(),
      transform.getRotation().y(),
      transform.getRotation().z(),
      transform.getRotation().w());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //yaw = yaw*180/3.14159265;

  std::string dataSend;
  dataSend = "{\"header\":[" + std::to_string(scan->angle_min) + "," + 
                   std::to_string(scan->angle_max) + "," + 
                   std::to_string(scan->angle_increment) + "," + 
                   std::to_string(scan->time_increment) + "," + 
                   std::to_string(scan->scan_time) + "," + 
                   std::to_string(scan->range_min) + "," + 
                   std::to_string(scan->range_max) + "," + 
                   std::to_string(transform.getOrigin().x()) + "," + 
                   std::to_string(transform.getOrigin().y()) + "," + 
                   std::to_string(transform.getOrigin().z()) + "," + 
                   std::to_string(yaw) + "],\"laser\":[";
                   
                   

  uint32_t laserDataSize = scan->ranges.size();
  for(uint32_t i = 0; i < laserDataSize; i++){
    std::stringstream temp;
    temp << std::fixed << std::setprecision(3) << scan->ranges[i];
    std::string s = temp.str();

    if(s != "inf"){
      dataSend += s;
    }
    else{
      dataSend += "-1";
    }
    
    if((i + 1) < laserDataSize){
      dataSend += ",";
    }
  }

  dataSend += "]}";
  //std::count<<dataSend;
  //std::count<<"\n";
  laserData = dataSend;
}

SocketTcpParameter InitTcpSocket(uint16_t port, uint16_t timeoutMs){
  std::string killPortStr = "sudo kill -9 `sudo lsof -t -i:";
  killPortStr = killPortStr + std::to_string(port) + "`";
  system(killPortStr.c_str());

  SocketTcpParameter parameter;

  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeoutMs*1000;
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
  if(timeoutMs > 0){
    //if (setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO,
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
                                                &tv, sizeof(struct timeval)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
  }
  else{
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
  }
  parameter.Address.sin_family = AF_INET;
  parameter.Address.sin_addr.s_addr = INADDR_ANY;
  parameter.Address.sin_port = htons( port );

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr *)&parameter.Address, 
                                sizeof(parameter.Address))<0)
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

  parameter.ServerFd = server_fd;
  return parameter;
}
