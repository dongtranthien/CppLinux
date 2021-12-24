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

#define PORT_COMMUNICATE_WITH_APP                     2001
#define PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL      2004
#define PORT_COMMUNICATE_WITH_NODE_SEND_MAP           2005
#define PORT_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS 2006

void ThreadStartSendMapNode();
void CommunicateWithApp();
void CheckCommunicateWithRosLaunchControlDisconnect();
void CheckCommunicateWithNodeSendMap();
void CheckCommunicateWithNodeSendLidarAndPos();
void RosRunning();


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
  SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS,
  SERVER_FD_COMMUNICATE_TOTAL
}ServerFileDescription;
typedef struct{
  int ServerFd;
  struct sockaddr_in Address;
  bool IsConnect = false;
  int Socket;
}SocketTcpParameter;
SocketTcpParameter socketTcpParameter[SERVER_FD_COMMUNICATE_TOTAL];
bool isSendInitialPose = false;
double angle, posX, posY;

SocketTcpParameter InitTcpSocket(uint16_t port, uint16_t timeoutMs);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AgvRosMainProcess");

  //socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP] = InitTcpSocket(PORT_COMMUNICATE_WITH_APP, 0);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL] = InitTcpSocket(PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL, 100);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP] = InitTcpSocket(PORT_COMMUNICATE_WITH_NODE_SEND_MAP, 100);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS] = InitTcpSocket(PORT_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS, 100);

  std::thread t1(CommunicateWithApp);
  std::thread t2(CheckCommunicateWithRosLaunchControlDisconnect);
  std::thread t3(CheckCommunicateWithNodeSendMap);
  std::thread t4(CheckCommunicateWithNodeSendLidarAndPos);
  std::thread t5(RosRunning);
  
  t1.join();
  t2.join();
  t3.join();
  t3.join();
  t4.join();
  t5.join();

  return 0;
}

void ThreadStartSendMapNode(){
  system("/home/idea/Documents/CppLinux/002_RosSendMapPixelTcp/buid/main");
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
      
      if(valread > 0){
        //if((buffer[0] == 0x01)&&(buffer[1] == 0x02)&&(buffer[2] == 0x02)&&(buffer[3] == 0x02)){
        if(strncmp("{StartMapping}", buffer, 14) == 0){
          //rosLaunchCommand = ROS_LAUNCH_COMMAND_MAPPING;
          send(new_socket, "Ok-App-StartMapping\n", 7, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStartMapping", 21, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].Socket, "{StartSendMap}", 14, 0);

        }
        //else if((buffer[0] == 0x01)&&(buffer[1] == 0x03)&&(buffer[2] == 0x02)&&(buffer[3] == 0x02)){
        else if(strncmp("{StopMapping}", buffer, 13) == 0){
          send(new_socket, "Ok-App-StopMapping\n", 19, 0);
          system("rosrun map_server map_saver -f /home/idea/mapAgv/map");
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStopMapping", 20, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].Socket, "{StopSendMap}", 13, 0);
        }
        else if(strncmp("{StartLocation}", buffer, 15) == 0){
          send(new_socket, "Ok-App-StartLocation\n", 21, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStartLocation", 22, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].Socket, "RosSendLidarAndPosStart", 23, 0);
        }
        else if(strncmp("{StopLocation}", buffer, 14) == 0){
          send(new_socket, "Ok-App-StartLocation\n", 21, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStopLocation", 21, 0);
          send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].Socket, "RosSendLidarAndPosStop", 22, 0);
        }
        // Add ping signal from app -> to check connect/loss connect/close app
        else if((strncmp("{initialpose,", buffer, 13) == 0)){
          send(new_socket, "Ok-App-initialpose\n", 19, 0);

          uint8_t index_t = 13;
          uint8_t lengthMax = 9;
          bool isError = false;
          char str_t[10] = {0};
          uint8_t indexStart_t = index_t;
          while((index_t - indexStart_t) < lengthMax){
            if((buffer[index_t] == '-')||(buffer[index_t] == '.')||(buffer[index_t] == ',')||((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))){
              if(buffer[index_t] == ','){
                break;
              }
              str_t[index_t - indexStart_t] = buffer[index_t];
            }
            else{
              // Error
              isError = true;
              std::cout << "Received initialpose error - angle";
              break;
            }
            index_t++;
          }

          if(isError||((index_t - indexStart_t) == lengthMax)){
            // Error
          }
          else{
            angle = std::stod(str_t);

            index_t++;
            indexStart_t = index_t;
            lengthMax = 10;
            while((index_t - indexStart_t) < lengthMax){
              if((buffer[index_t] == '-')||(buffer[index_t] == '.')||(buffer[index_t] == ',')||((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))){
                if(buffer[index_t] == ','){
                  break;
                }
                str_t[index_t - indexStart_t] = buffer[index_t];
              }
              else{
                // Error
                isError = true;
                std::cout << "Received initialpose error - posX";
                break;
              }
              index_t++;
            }

            if(isError||((index_t - indexStart_t) == lengthMax)){
              // Error
            }
            else{
              posX = std::stod(str_t);

              index_t++;
              indexStart_t = index_t;
              lengthMax = 10;
              while((index_t - indexStart_t) < lengthMax){
                if((buffer[index_t] == '-')||(buffer[index_t] == '.')||(buffer[index_t] == '}')||((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))){
                  if(buffer[index_t] == '}'){
                    break;
                  }
                  str_t[index_t - indexStart_t] = buffer[index_t];
                }
                else{
                  // Error
                  isError = true;
                  std::cout << "Received initialpose error - posY";
                  break;
                }
                index_t++;
              }

              if(isError||((index_t - indexStart_t) == lengthMax)){
                // Error
              }
              else{
                posY = std::stod(str_t);

                std::cout << "initialpose - " + std::to_string(angle) + "-" + std::to_string(posX) + "-" + std::to_string(posY);
                isSendInitialPose = true;
              }
            }
          }
          
        }
        // Add ping signal from app -> to check connect/loss connect/close app
        
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

void CheckCommunicateWithNodeSendMap(){
  struct sockaddr_in address_t;
  int addrlen = sizeof(address_t);
  char buffer[1024] = {0};
  int new_socket;

  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].IsConnect = false;
  while(true){
    if(!socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].IsConnect){
      if ((new_socket = accept(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].ServerFd, 
                        (struct sockaddr *)&socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].Address, 
                        (socklen_t*)&addrlen))<0)
      {
        //perror("accept");
        //exit(EXIT_FAILURE);
        
      }
      else{
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].IsConnect = true;
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].Socket = new_socket;

        std::cout<<"NodeSendMap Connect..\n";
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].IsConnect = false;
        std::cout<<"NodeSendMap Disconnect..\n";
        //while(1);
      }
    }
  }
}

void CheckCommunicateWithNodeSendLidarAndPos(){
  struct sockaddr_in address_t;
  int addrlen = sizeof(address_t);
  char buffer[1024] = {0};
  int new_socket;

  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].IsConnect = false;
  while(true){
    if(!socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].IsConnect){
      if ((new_socket = accept(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].ServerFd, 
                        (struct sockaddr *)&socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].Address, 
                        (socklen_t*)&addrlen))<0)
      {
        //perror("accept");
        //exit(EXIT_FAILURE);
        
      }
      else{
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].IsConnect = true;
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].Socket = new_socket;

        std::cout<<"NodeSendLidarAndPos Connect..\n";
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      printf("%s\n",buffer );
      printf("%d\n",valread );

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].IsConnect = false;
        std::cout<<"NodeSendLidarAndPos Disconnect..\n";
        //while(1);
      }
      else{
        if(valread == 17){
          if(strncmp(buffer, "OverDelayWaitData", 17) == 0){
            if(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect){
              send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStartLocation", 22, 0);
            }
          }
        }
        else if(valread == 28){
          if(strncmp(buffer, "OverWaitDataWhenStartProgram", 28) == 0){
            if(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect){
              send(socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].Socket, "RosLaunchStartLocation", 22, 0);
            }
          }
        }
      }
    }
  }
}

void RosRunning(){
  ros::NodeHandle nh_;
  ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
  ros::Rate loop_rate(10); // 10HZ = 0.1s
  ros::spinOnce();

  while (ros::ok()){
    if(isSendInitialPose){
      isSendInitialPose = false;

      std::string fixed_frame = "map";
      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.frame_id = fixed_frame;
      pose.header.stamp = ros::Time::now();

      // set x,y coord
      pose.pose.pose.position.x = posX;
      pose.pose.pose.position.y = posY;
      pose.pose.pose.position.z = 0.0;

      // set theta
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, angle);
      tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
      pose.pose.covariance[6*0+0] = 0.5 * 0.5;
      pose.pose.covariance[6*1+1] = 0.5 * 0.5;
      pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

      // publish
      //ROS_INFO("x: %f, y: %f, z: 0.0, theta: %f",x,y,theta);
      pub_.publish(pose);
    }
    
    loop_rate.sleep(); 
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
