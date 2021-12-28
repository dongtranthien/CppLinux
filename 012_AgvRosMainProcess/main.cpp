#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"

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
#include <fstream>
#include <cmath>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "crc.h"
#include "json.h"

#define PORT_COMMUNICATE_WITH_APP                     2001
#define PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL      2004
#define PORT_COMMUNICATE_WITH_NODE_SEND_MAP           2005
#define PORT_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS 2006
#define DEBUG_CONTROL_TO_LANDMARK

void ThreadStartSendMapNode();
void CommunicateWithApp();
void CheckCommunicateWithRosLaunchControlDisconnect();
void CheckCommunicateWithNodeSendMap();
void CheckCommunicateWithNodeSendLidarAndPos();
void RosRunning();
void ControlDemo();
void ControlToLandmarkProcess();


std::string exec(const char* cmd);

using json = nlohmann::json;

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

typedef enum{
  GO_TO_LANDMARK_PROCESS_STEP_IDLE = 0,
  GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME
}GotoLandmarkProcessStep;

typedef enum{
  GO_TO_LANDMARK_FIRST_TIME_STEP_CALCULATE_ANGLE_ROBOT_TO_LANDMARK = 0,
  GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK,
  GO_TO_LANDMARK_FIRST_TIME_STEP_GOTO_LANDMARK,
  GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_WITH_DIRECT_SAME_LANDMARK
}GotoLandmarkFirstTimeStep;

SocketTcpParameter socketTcpParameter[SERVER_FD_COMMUNICATE_TOTAL];
bool isSendInitialPose = false;
double angle, posX, posY;
bool isReceivedPathAndLandmark = false, isReceivedDonePathAndLandmark = false;
json pathAndLandmarkJson;
bool isPathAndLandmarkJsonDataOk = false;
uint8_t indexLMCurrent = 255, indexLMToRun;
bool isReceivedLMToRun = false;
GotoLandmarkProcessStep gotoLandmarkProcessStep;
GotoLandmarkFirstTimeStep gotoLandmarkFirstTimeStep;
double xRobot, yRobot, yawRobot, vxControl = 0, vThControl = 0, xToRun, yToRun, angleRobotToLandmark, angleLandmark;
bool isNewReceivedRobotData = false;

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
  std::thread t6(ControlDemo);
  std::thread t7(ControlToLandmarkProcess);
  
  t1.join();
  t2.join();
  t3.join();
  t4.join();
  t5.join();
  t6.join();
  t7.join();

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
  char buffer[8192] = {0};
  char pathAndLandmarkData[8192] = {0};
  int pathAndLandmarkDataLength = 0;

  #ifdef DEBUG
    printf("\nTcpRunning...");
  #endif

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
      perror("socket failed");
      exit(EXIT_FAILURE);
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  //setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
  // Forcefully attaching socket to the port 8080
  if (setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv))
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
        //perror("accept");
        //exit(EXIT_FAILURE);
      }
      else{
        isClientConnect = true;
        std::cout << "CommunicateWithApp Connect\n";
      }
    }
    else{
      valread = read( new_socket , buffer, 8019);
      if(valread != (-1)){
        buffer[valread] = 0;
        printf("\nBuffer: %s\n",buffer );
        printf("%d\n",valread );
      }

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
        else if((strncmp("{\"pathAndLandmark\":", buffer, 19) == 0)){
          isReceivedDonePathAndLandmark = false;
          
          if(!isReceivedPathAndLandmark){
            isReceivedPathAndLandmark = true;
            //send(new_socket, buffer, valread, 0);
            for(uint32_t i = 0; i < valread; i++){
              pathAndLandmarkData[i] = buffer[i];
            }
            pathAndLandmarkDataLength = valread;
          }
        }
        else if((strncmp("{\"responsePathAndLandmark\":\"Ok\"}", buffer, 32) == 0)){
          std::cout<<"Received Ok.....\n";
          if(isReceivedDonePathAndLandmark){
            isReceivedDonePathAndLandmark = false;

            std::ofstream file("pathAndLandmark.txt");
            file << pathAndLandmarkData;
            file.close();

            try{
              pathAndLandmarkJson = json::parse(pathAndLandmarkData);
              isPathAndLandmarkJsonDataOk = true;
            }
            catch (json::parse_error& ex){
              std::cerr << "JSON parse error at byte " << ex.byte << std::endl;
            }

            //std::cout<<"\nData test: ";
            //std::cout<<pathAndLandmarkJson["pathAndLandmark"]["path"]["total"];
            //std::cout<<"\n";
            //std::cout<<pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][0]["n"];
            //std::cout<<"\n";
            if(isPathAndLandmarkJsonDataOk){

            }
          }
        }
        else if((strncmp("{\"responsePathAndLandmark\":\"Error\"}", buffer, 35) == 0)){

        }
        else if((strncmp("{\"sendLandmark\":", buffer, 16) == 0)){  // Error when landmark index > 9
          isReceivedLMToRun = true;
          indexLMToRun = buffer[16] - '0';

          send(new_socket, "{\"reponseSendLandmark\":\"Ok\"}", 28, 0);
        }
        else{
          if(isReceivedPathAndLandmark){
            isReceivedDonePathAndLandmark = false;

            for(uint32_t i = 0; i < valread; i++){
              if((pathAndLandmarkDataLength + i) >= 8192) break;
              pathAndLandmarkData[pathAndLandmarkDataLength + i] = buffer[i];
            }
            pathAndLandmarkDataLength += valread;
          }
        }
        // Add ping signal from app -> to check connect/loss connect/close app
        
      }
      else if(valread == 0){
        isClientConnect = false;

        std::cout << "CommunicateWithApp Disconnect\n";
      }
      else if(valread == (-1)){
        if(isReceivedPathAndLandmark){
          isReceivedPathAndLandmark = false;
          isReceivedDonePathAndLandmark = true;

          std::cout << "\nRead data next......\n";
          
          send(new_socket, pathAndLandmarkData, pathAndLandmarkDataLength, 0);
        }
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
      //printf("%s\n",buffer );
      //printf("%d\n",valread );

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
        if((strncmp(buffer, "{RobotPositionMm:", 17) == 0)&&(buffer[valread - 1] == '}')){
          uint8_t index_t = 17, lengthReceived = 0;
          bool isError = false;
          char strArr_t[13] = {0};
          while(lengthReceived < 12){
            if(index_t >= valread){
              isError = true;
              break;
            }
            else{
              if(((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))||(buffer[index_t] == '.')||(buffer[index_t] == '-')){
                strArr_t[lengthReceived] = buffer[index_t];
                lengthReceived++;
                index_t++;
              }
              else if(buffer[index_t] == ','){
                strArr_t[lengthReceived] = 0;
                index_t++;
                break;
              }
              else{
                isError = true;
                break;
              }
            }
          }

          if((!isError)||(lengthReceived >= 12)){
            std::string str_t;
            str_t = strArr_t;
            //std::cout << "xRobot received: " << str_t << "\n";
            xRobot = std::stof(str_t);

            lengthReceived = 0;
            while(lengthReceived < 12){
              if(index_t >= valread){
                isError = true;
                break;
              }
              else{
                if(((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))||(buffer[index_t] == '.')||(buffer[index_t] == '-')){
                  strArr_t[lengthReceived] = buffer[index_t];
                  lengthReceived++;
                  index_t++;
                }
                else if(buffer[index_t] == ','){
                  strArr_t[lengthReceived] = 0;
                  index_t++;
                  break;
                }
                else{
                  isError = true;
                  break;
                }
              }
            }

            if((!isError)||(lengthReceived >= 12)){
              str_t = strArr_t;
              //std::cout << "yRobot received: " << str_t << "\n";
              yRobot = std::stof(str_t);

              lengthReceived = 0;
              while(lengthReceived < 12){
                if(index_t >= valread){
                  isError = true;
                  break;
                }
                else{
                  if(((buffer[index_t] >= '0')&&(buffer[index_t] <= '9'))||(buffer[index_t] == '.')||(buffer[index_t] == '-')){
                    strArr_t[lengthReceived] = buffer[index_t];
                    lengthReceived++;
                    index_t++;
                  }
                  else if(buffer[index_t] == '}'){
                    strArr_t[lengthReceived] = 0;
                    index_t++;
                    break;
                  }
                  else{
                    isError = true;
                    break;
                  }
                }
              }

              if((!isError)||(lengthReceived >= 12)){
                str_t = strArr_t;
                //std::cout << "yawRobot received: " << str_t << "\n";
                yawRobot = std::stod(str_t);
                if(yawRobot < 0){
                  yawRobot = 6.283185 + yawRobot;
                }

                //std::cout<< "Received robot position: " << xRobot << "-" << yRobot << "-" << yawRobot << "\n";
                isNewReceivedRobotData = true;
              }
              else{
                std::cout<<"Received yaw robot failed\n";
              }
              
            }
            else{
              std::cout<<"Received y robot failed\n";
            }
          }
          else{
            std::cout<<"Received x robot failed\n";
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

void ControlDemo(){
  ros::NodeHandle n;
  ros::Publisher cmdVel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);

  while(n.ok()){
    ros::spinOnce();               // check for incoming messages

    geometry_msgs::Twist msg;
    msg.linear.x = vxControl;
    msg.angular.z = vThControl;

    //publish the message
    cmdVel_pub.publish(msg);

    r.sleep();
  }
}

void ControlToLandmarkProcess(){
  while(true){
    if(gotoLandmarkProcessStep == GO_TO_LANDMARK_PROCESS_STEP_IDLE){
      if(isReceivedLMToRun){
        isReceivedLMToRun = false;
        if(isPathAndLandmarkJsonDataOk){
          if(indexLMCurrent == 255){
            xToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["p"][0];
            yToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["p"][1];
            gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME;
            gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_CALCULATE_ANGLE_ROBOT_TO_LANDMARK;
            #ifdef DEBUG_CONTROL_TO_LANDMARK
              std::cout<< "Position landmark to run: " << xToRun << "-" << yToRun << "\n";
            #endif
          }
          else{

          }
        }
        else{
          std::cout<<"Path and landmark error -> Not run to landmark\n";
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else if(gotoLandmarkProcessStep == GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME){
      // Get position of landmark to run
      switch(gotoLandmarkFirstTimeStep){
        case GO_TO_LANDMARK_FIRST_TIME_STEP_CALCULATE_ANGLE_ROBOT_TO_LANDMARK:{
          if((xRobot == xToRun)&&(yRobot == yToRun)){
            gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
          }
          else{
            if(yRobot == yToRun){
              if(xRobot < xToRun){
                angleRobotToLandmark = 0;
              }
              else{
                angleRobotToLandmark = 3.141592;
              }
            }
            else if(xRobot == xToRun){
              if(yRobot < yToRun){
                angleRobotToLandmark = 1.570796;
              }
              else{
                angleRobotToLandmark = 4.712388;
              }
            }
            else if((xRobot < xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan((yToRun - yRobot)/(xToRun - xRobot));
            }
            else if((xRobot > xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 - angleRobotToLandmark;
            }
            else if((xRobot > xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 + angleRobotToLandmark;
            }
            else if((xRobot < xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 6.283185 - angleRobotToLandmark;
            }

            double deltaAngle = abs((double)(yawRobot - angleRobotToLandmark));

            if(yawRobot < angleRobotToLandmark){
              if(deltaAngle <= 3.141592){
                vThControl = 0.1;
              }
              else{
                vThControl = -0.1;
              }
            }
            else{
              if(deltaAngle <= 3.141592){
                vThControl = -0.1;
              }
              else{
                vThControl = 0.1;
              }
            }

            std::cout << "yawRobot: " << yawRobot << "\n";
            std::cout << "angleRobotToLandmark: " << angleRobotToLandmark << "\n";
            std::cout << "deltaAngle: " << deltaAngle << "\n";
            std::cout << "vThControl: " << vThControl << "\n";
            std::cout << "Angle robot to landmark: " << (angleRobotToLandmark*360/6.283185) << "\n";
            gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK;
          }
          break;
        }
        case GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK:{
          yawRobot = yawRobot + (vThControl*20/1000);
          //std::cout << "yawRobot: " << yawRobot << "\n";

          if(isNewReceivedRobotData){
            isNewReceivedRobotData = false;
            if(yRobot == yToRun){
              if(xRobot < xToRun){
                angleRobotToLandmark = 0;
              }
              else{
                angleRobotToLandmark = 3.141592;
              }
            }
            else if(xRobot == xToRun){
              if(yRobot < yToRun){
                angleRobotToLandmark = 1.570796;
              }
              else{
                angleRobotToLandmark = 4.712388;
              }
            }
            else if((xRobot < xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan((yToRun - yRobot)/(xToRun - xRobot));
            }
            else if((xRobot > xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 - angleRobotToLandmark;
            }
            else if((xRobot > xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 + angleRobotToLandmark;
            }
            else if((xRobot < xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 6.283185 - angleRobotToLandmark;
            }
          }
          
          double deltaAngle = abs((double)(yawRobot - angleRobotToLandmark));
          //std::cout << "deltaAngle: " << deltaAngle << "\n";
          //if(deltaAngle < (1*3.141592/180)){
          if(deltaAngle < (0.017)){
            vThControl = 0;
            gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_GOTO_LANDMARK;
            #ifdef DEBUG_CONTROL_TO_LANDMARK
              std::cout << "Go to landmark first time - rotate done\n";
            #endif
          }
          break;
        }
        case GO_TO_LANDMARK_FIRST_TIME_STEP_GOTO_LANDMARK:{
          if(isNewReceivedRobotData){
            isNewReceivedRobotData = false;
            if(yRobot == yToRun){
              if(xRobot < xToRun){
                angleRobotToLandmark = 0;
              }
              else{
                angleRobotToLandmark = 3.141592;
              }
            }
            else if(xRobot == xToRun){
              if(yRobot < yToRun){
                angleRobotToLandmark = 1.570796;
              }
              else{
                angleRobotToLandmark = 4.712388;
              }
            }
            else if((xRobot < xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan((yToRun - yRobot)/(xToRun - xRobot));
            }
            else if((xRobot > xToRun)&&(yRobot < yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 - angleRobotToLandmark;
            }
            else if((xRobot > xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 3.141592 + angleRobotToLandmark;
            }
            else if((xRobot < xToRun)&&(yRobot > yToRun)){
              angleRobotToLandmark = atan(abs(yToRun - yRobot)/abs(xToRun - xRobot));
              angleRobotToLandmark = 6.283185 - angleRobotToLandmark;
            }
          }
          
          double deltaAngle = abs((double)(yawRobot - angleRobotToLandmark));
          if(deltaAngle < (0.017)){
            vThControl = 0;
          }
          else if(yawRobot < angleRobotToLandmark){
            if(deltaAngle <= 3.141592){
              vThControl = 0.1;
            }
            else{
              vThControl = -0.1;
            }
          }
          else{
            if(deltaAngle <= 3.141592){
              vThControl = -0.1;
            }
            else{
              vThControl = 0.1;
            }
          }

          vxControl = 0.1;

          if(((xRobot >= (xToRun - 0.05))&&(xRobot <= (xToRun + 0.05)))&&((yRobot >= (yToRun - 0.05))&&(yRobot <= (yToRun + 0.05)))){
            vxControl = 0;
            vThControl = 0;
            #ifdef DEBUG_CONTROL_TO_LANDMARK
              std::cout << "Go to landmark first time - Robot move to point of landmark done\n";
            #endif

            int indexPath = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["i"];
            double x1 = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][indexPath]["d"][0];
            double y1 = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][indexPath]["d"][1];
            double x2 = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][indexPath]["d"][2];
            double y2 = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][indexPath]["d"][3];
            double x0, y0;

            if((xToRun == x1)&&(yToRun == y1)){
              x0 = x2;
              y0 = y2;
            }
            else if((xToRun == x2)&&(yToRun == y2)){
              x0 = x1;
              y0 = y1;
            }
            else{
              std::cout << "Position of landmark no in any path\n";
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
              break;
            }
            // swap
            x1 = x0;
            x0 = xToRun;
            xToRun = x1;
            y1 = y0;
            y0 = yToRun;
            yToRun = y1;

            if(y0 == yToRun){
              if(x0 < xToRun){
                angleLandmark = 0;
              }
              else{
                angleLandmark = 3.141592;
              }
            }
            else if(x0 == xToRun){
              if(y0 < yToRun){
                angleLandmark = 1.570796;
              }
              else{
                angleLandmark = 4.712388;
              }
            }
            else if((x0 < xToRun)&&(y0 < yToRun)){
              angleLandmark = atan((yToRun - y0)/(xToRun - x0));
            }
            else if((x0 > xToRun)&&(y0 < yToRun)){
              angleLandmark = atan(abs(yToRun - y0)/(xToRun - x0));
              angleLandmark = 3.141592 - angleLandmark;
            }
            else if((x0 > xToRun)&&(y0 > yToRun)){
              angleLandmark = atan(abs(yToRun - y0)/abs(xToRun - x0));
              angleLandmark = 3.141592 + angleLandmark;
            }
            else if((x0 < xToRun)&&(y0 > yToRun)){
              angleLandmark = atan(abs(yToRun - y0)/abs(xToRun - x0));
              angleLandmark = 6.283185 - angleLandmark;
            }

            double deltaAngle = abs((double)(yawRobot - angleLandmark));

            if(yawRobot < angleLandmark){
              if(deltaAngle <= 3.141592){
                vThControl = 0.1;
              }
              else{
                vThControl = -0.1;
              }
            }
            else{
              if(deltaAngle <= 3.141592){
                vThControl = -0.1;
              }
              else{
                vThControl = 0.1;
              }
            }

            gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_WITH_DIRECT_SAME_LANDMARK;
          }
          break;
        }
        case GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_WITH_DIRECT_SAME_LANDMARK:{
          double deltaAngle = abs((double)(yawRobot - angleLandmark));
          yawRobot = yawRobot + (vThControl*20/1000);

          if(deltaAngle < (0.017)){
            vThControl = 0;
            gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
            #ifdef DEBUG_CONTROL_TO_LANDMARK
              std::cout << "Go to landmark first time - go to landmark done\n";
            #endif
          }

          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
