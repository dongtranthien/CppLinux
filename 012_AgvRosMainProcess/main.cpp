#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
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

#include "crc.h"

#define PORT_COMMUNICATE_WITH_APP                   2001
#define PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL    2004
#define PORT_COMMUNICATE_WITH_NODE_SEND_MAP         2005

void CommunicateWithApp();
void CheckCommunicateWithRosLaunchControlDisconnect();


std::string exec(const char* cmd);

typedef enum{
  ROS_LAUNCH_COMMAND_NONE = 0,
  ROS_LAUNCH_COMMAND_MAPPING,
  ROS_LAUNCH_COMMAND_LOCALIZATION
}RosLaunchCommand;
RosLaunchCommand rosLaunchCommand = ROS_LAUNCH_COMMAND_NONE;
RosLaunchCommand rosLaunchCommandPre = ROS_LAUNCH_COMMAND_NONE;

typedef enum{
  SERVER_FD_COMMUNICATE_WITH_APP = 0,
  SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL,
  SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP,
  SERVER_FD_COMMUNICATE_TOTAL
}ServerFileDescription;
typedef struct{
  int ServerFd;
  struct sockaddr_in Address;
  bool IsConnect = false;
  int Socket;
}SocketTcpParameter;
SocketTcpParameter socketTcpParameter[SERVER_FD_COMMUNICATE_TOTAL];

SocketTcpParameter InitTcpSocket(uint16_t port, uint16_t timeoutMs);
int TcpSocketSend(ServerFileDescription type, void *buffer, size_t n);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AgvRosMainProcess");

  //socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP] = InitTcpSocket(PORT_COMMUNICATE_WITH_APP, 0);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL] = InitTcpSocket(PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL, 100);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP] = InitTcpSocket(PORT_COMMUNICATE_WITH_NODE_SEND_MAP, 100);

  std::thread t1(CommunicateWithApp);
  std::thread t2(CheckCommunicateWithRosLaunchControlDisconnect);

  t1.join();
  t2.join();

  std::cout << "Asdf";

  

  return 0;
}

void CommunicateWithApp(){
  std::string killPortStr = "sudo kill -9 `sudo lsof -t -i:";
  killPortStr = killPortStr + std::to_string(PORT_COMMUNICATE_WITH_APP) + "`";
  system(killPortStr.c_str());
  
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
  address.sin_port = htons( PORT_COMMUNICATE_WITH_APP );

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
      else{
        isClientConnect = true;
        std::cout << "CommunicateWithApp Connect\n";
      }
    }
    else{
      valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );

      //TcpSocketSend(SERVER_FD_COMMUNICATE_WITH_APP, buffer, valread);
      
      if(valread == 4){
        if((buffer[0] == 0x01)&&(buffer[1] == 0x02)&&(buffer[2] == 0x02)&&(buffer[3] == 0x02)){
          //rosLaunchCommand = ROS_LAUNCH_COMMAND_MAPPING;
          send(new_socket, "Ok-App-StartMapping\n", 7, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStartMapping", 21, 0);

        }
        else if((buffer[0] == 0x01)&&(buffer[1] == 0x03)&&(buffer[2] == 0x02)&&(buffer[3] == 0x02)){
          send(new_socket, "Ok-App-StopMapping\n", 7, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStopMapping", 20, 0);
        }
      }
      else if(valread == 0){
        isClientConnect = false;

        std::cout << "CommunicateWithApp Disconnect\n";
      }
    }
  }
}

void CheckCommunicateWithRosLaunchControlDisconnect(){
  struct sockaddr_in address_t;
  int addrlen = sizeof(address_t);
  char buffer[1024] = {0};
  int new_socket;

  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect = false;
  while(true){
    if(!socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect){
      if ((new_socket = accept(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].ServerFd, (struct sockaddr *)&socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Address, 
                       (socklen_t*)&addrlen))<0)
      {
        //perror("accept");
        //exit(EXIT_FAILURE);
        
      }
      else{
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect = true;
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket = new_socket;

        std::cout<<"RosLaunchControl Connect..\n";
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect = false;
        std::cout<<"RosLaunchControl Disconnect..\n";
        //while(1);
      }
    }
  }
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

int TcpSocketSend(ServerFileDescription type, void *buffer, size_t n){
  
}

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
        std::cout << buffer.data();
    }
    return result;
}
