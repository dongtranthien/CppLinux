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
#define PORT_COMMUNICATE_WITH_APP_MANUAL_CONTROL      2003
#define PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL      2004
#define PORT_COMMUNICATE_WITH_NODE_SEND_MAP           2005
#define PORT_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS 2006
#define DEBUG_CONTROL_TO_LANDMARK
#define CYCLE_TIME_SEND_MS				                    20
#define RUN_SIMULATE_WITH_TURTLE_BOT

void ThreadStartSendMapNode();
void CommunicateWithApp();
void CheckCommunicateWithRosLaunchControlDisconnect();
void CheckCommunicateWithNodeSendMap();
void CheckCommunicateWithNodeSendLidarAndPos();
void RosRunning();
void ControlDemo();
void ControlToLandmarkProcess();
void SerialCommunicate();
void CommunicateWithAppToControlManual();


void CalculateVelocityOfManualControl(uint8_t controlValue, uint32_t *leftValue, uint32_t *rightValue);
void CalculateVelocityOfAutoControl(uint32_t *leftValue, uint32_t *rightValue);
std::string exec(const char* cmd);
double DegToRad(double val);
void ConfigLogFile();
uint64_t micros();

using json = nlohmann::json;
using namespace LibSerial;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

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
  GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME,
  GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK
}GotoLandmarkProcessStep;

typedef enum{
  GO_TO_LANDMARK_FIRST_TIME_STEP_CALCULATE_ANGLE_ROBOT_TO_LANDMARK = 0,
  GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK,
  GO_TO_LANDMARK_FIRST_TIME_STEP_GOTO_LANDMARK,
  GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_WITH_DIRECT_SAME_LANDMARK
}GotoLandmarkFirstTimeStep;

typedef enum{
  GO_TO_LANDMARK_STEP_FIND_PATH_TO_GO = 0,
  GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH,
  GO_TO_LANDMARK_STEP_RUNNING_PROCESS
}GotoLandmarkStep;

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
GotoLandmarkStep gotoLandmarkStep;
double xRobot, yRobot, yawRobot, vxControl = 0, vxSet, vThControl = 0, vThSet, xToRun, yToRun, angleRobotToLandmark, angleLandmark;
bool isNewReceivedRobotData = false;
uint8_t pathIdArrRunning[32], pathIdArrRunningLength, indexPathIdRunning;
double pointArrRunning[32][2];
uint8_t pointArrRunningLength, indexPointRunning;
double angleRobotToPoint;
uint32_t counterCheckWaitingReceivedFromTcp = 0, controlDat = 0;
SerialPort serial_port;
float velocityRunningFloat[2];
int counterReceivedNoConnect = 0;
uint32_t errorReceivedManualControl = 0;

SocketTcpParameter InitTcpSocket(uint16_t port, uint16_t timeoutMs);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AgvRosMainProcess");
  ConfigLogFile();

  //socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_APP] = InitTcpSocket(PORT_COMMUNICATE_WITH_APP, 0);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL] = InitTcpSocket(PORT_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL, 100);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP] = InitTcpSocket(PORT_COMMUNICATE_WITH_NODE_SEND_MAP, 100);
  socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS] = InitTcpSocket(PORT_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS, 100);

  std::thread t1(CommunicateWithApp);
  std::thread t2(CheckCommunicateWithRosLaunchControlDisconnect);
  std::thread t3(CheckCommunicateWithNodeSendMap);
  std::thread t4(CheckCommunicateWithNodeSendLidarAndPos);
  std::thread t5(RosRunning);
  #ifdef RUN_SIMULATE_WITH_TURTLE_BOT
    std::thread t6(ControlDemo);
  #endif
  std::thread t7(ControlToLandmarkProcess);
  #ifndef RUN_SIMULATE_WITH_TURTLE_BOT
    std::thread t8(SerialCommunicate);
    std::thread t9(CommunicateWithAppToControlManual);
  #endif
  
  t1.join();
  t2.join();
  t3.join();
  t4.join();
  t5.join();
  #ifdef RUN_SIMULATE_WITH_TURTLE_BOT
    t6.join();
  #endif
  t7.join();
  #ifndef RUN_SIMULATE_WITH_TURTLE_BOT
    t8.join();
    t9.join();
  #endif

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
    std::cout<<"TcpRunning..."<<std::endl;
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
    std::cout<<"listen..."<<std::endl;
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
        std::cout << "CommunicateWithApp Connect"<<std::endl;
      }
    }
    else{
      valread = read( new_socket , buffer, 8019);
      if(valread != (-1)){
        buffer[valread] = 0;
        std::cout<<"Buffer: "<<buffer<<std::endl;
        std::cout<<std::to_string(valread)<<std::endl;
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
              std::cout << "Received initialpose error - angle"<<std::endl;
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
                std::cout << "Received initialpose error - posX"<<std::endl;
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
                  std::cout << "Received initialpose error - posY"<<std::endl;
                  break;
                }
                index_t++;
              }

              if(isError||((index_t - indexStart_t) == lengthMax)){
                // Error
              }
              else{
                posY = std::stod(str_t);

                std::cout << "initialpose - " + std::to_string(angle) + "-" + std::to_string(posX) + "-" + std::to_string(posY)<<std::endl;
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
          std::cout<<"Received Ok....."<<std::endl;
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
            //std::cout<<""<<std::endl;
            //std::cout<<pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][0]["n"];
            //std::cout<<""<<std::endl;
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

        std::cout << "CommunicateWithApp Disconnect"<<std::endl;
      }
      else if(valread == (-1)){
        if(isReceivedPathAndLandmark){
          isReceivedPathAndLandmark = false;
          isReceivedDonePathAndLandmark = true;

          std::cout << "\nRead data next......"<<std::endl;
          
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

        std::cout<<"RosLaunchControl Connect.."<<std::endl;
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      std::cout<<buffer<<std::endl;
      std::cout<<std::to_string(valread)<<std::endl;

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_ROS_LAUNCH_CONTROL].IsConnect = false;
        std::cout<<"RosLaunchControl Disconnect.."<<std::endl;
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

        std::cout<<"NodeSendMap Connect.."<<std::endl;
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      std::cout<<buffer<<std::endl;
      std::cout<<std::to_string(valread)<<std::endl;

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_MAP].IsConnect = false;
        std::cout<<"NodeSendMap Disconnect.."<<std::endl;
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

        std::cout<<"NodeSendLidarAndPos Connect.."<<std::endl;
      }
    }
    else{
      int valread = read( new_socket , buffer, 1024);
      //printf("%s\n",buffer );
      //printf("%d\n",valread );

      if(valread == 0){
        socketTcpParameter[SERVER_FD_COMMUNICATE_WITH_NODE_SEND_LIDAR_AND_POS].IsConnect = false;
        std::cout<<"NodeSendLidarAndPos Disconnect.."<<std::endl;
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
            //std::cout << "xRobot received: " << str_t << ""<<std::endl;
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
              //std::cout << "yRobot received: " << str_t << ""<<std::endl;
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
                //std::cout << "yawRobot received: " << str_t << ""<<std::endl;
                yawRobot = std::stod(str_t);
                if(yawRobot < 0){
                  yawRobot = 6.283185 + yawRobot;
                }

                std::cout<<"YawRobot:"<<yawRobot<<std::endl;

                //std::cout<< "Received robot position: " << xRobot << "-" << yRobot << "-" << yawRobot << ""<<std::endl;
                isNewReceivedRobotData = true;
              }
              else{
                std::cout<<"Received yaw robot failed"<<std::endl;
              }
              
            }
            else{
              std::cout<<"Received y robot failed"<<std::endl;
            }
          }
          else{
            std::cout<<"Received x robot failed"<<std::endl;
          }
        }
      }
    }
  }
}

void RosRunning(){
  ros::NodeHandle nh_;
  ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

  #ifndef RUN_SIMULATE_WITH_TURTLE_BOT
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double xRobotNoCare = 0, yRobotNoCare = 0, thRobotNoCare = 0;
  #endif

  ros::Rate loop_rate(50); // 10HZ = 0.02s
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
    
    #ifndef RUN_SIMULATE_WITH_TURTLE_BOT
      current_time = ros::Time::now();
      float vLeft_t = velocityRunningFloat[0];
      float vRight_t = velocityRunningFloat[1];
      vLeft_t = vLeft_t/10000;
      vRight_t = vRight_t/10000;
      double vx = (vLeft_t + vRight_t)/2*(1);
      double vth = (-vRight_t + vLeft_t)/0.4;
      double vy = 0;

      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double delta_x = (vx * cos(thRobotNoCare) - vy * sin(thRobotNoCare)) * dt;
      double delta_y = (vx * sin(thRobotNoCare) + vy * cos(thRobotNoCare)) * dt;
      double delta_th = vth * dt;

      xRobotNoCare += delta_x;
      yRobotNoCare += delta_y;
      thRobotNoCare += delta_th;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(thRobotNoCare);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      odom_trans.transform.translation.x = xRobotNoCare;
      odom_trans.transform.translation.y = yRobotNoCare;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = xRobotNoCare;
      odom.pose.pose.position.y = yRobotNoCare;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;

      ros::spinOnce();
    #endif

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
  uint64_t startTime = micros();
  double deltaAnglePre;
  bool isGetFirstDeltaAngle = false;
  while(true){
    switch(gotoLandmarkProcessStep){
      case GO_TO_LANDMARK_PROCESS_STEP_IDLE:{
        if(isReceivedLMToRun){
          isReceivedLMToRun = false;
          if(isPathAndLandmarkJsonDataOk){
            if(indexLMCurrent == 255){
              xToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["p"][0];
              yToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["p"][1];
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME;
              gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_CALCULATE_ANGLE_ROBOT_TO_LANDMARK;
              #ifdef DEBUG_CONTROL_TO_LANDMARK
                std::cout<< "Position landmark to run: " << xToRun << "-" << yToRun << ""<<std::endl;
              #endif
            }
            else if(indexLMToRun != indexLMCurrent){
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK;
              gotoLandmarkStep = GO_TO_LANDMARK_STEP_FIND_PATH_TO_GO;
            }
          }
          else{
            std::cout<<"Path and landmark error -> Not run to landmark"<<std::endl;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        break;
      }
      case GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK_FIRST_TIME:{
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

              double deltaAngleAbs = abs((double)(yawRobot - angleRobotToLandmark));
              deltaAnglePre = ((double)(yawRobot - angleRobotToLandmark));
              if(yawRobot < angleRobotToLandmark){
                if(deltaAngleAbs <= 3.141592){
                  vThControl = 0.1;
                }
                else{
                  vThControl = -0.1;
                }
              }
              else{
                if(deltaAngleAbs <= 3.141592){
                  vThControl = -0.1;
                }
                else{
                  vThControl = 0.1;
                }
              }

              std::cout << "yawRobot: " << yawRobot << ""<<std::endl;
              std::cout << "angleRobotToLandmark: " << angleRobotToLandmark << ""<<std::endl;
              std::cout << "deltaAngleAbs: " << deltaAngleAbs << ""<<std::endl;
              std::cout << "vThControl: " << vThControl << ""<<std::endl;
              std::cout << "Angle robot to landmark: " << (angleRobotToLandmark*360/6.283185) << ""<<std::endl;
              gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK;
            }
            break;
          }
          case GO_TO_LANDMARK_FIRST_TIME_STEP_ROTATE_TO_LANDMARK:{
            yawRobot = yawRobot + (vThControl*20/1000);
            //std::cout << "yawRobot: " << yawRobot << ""<<std::endl;

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
            
            double deltaAngleAbs = abs((double)(yawRobot - angleRobotToLandmark));
            double deltaAngle = ((double)(yawRobot - angleRobotToLandmark));

            if(yawRobot < angleRobotToLandmark){
              if(deltaAngleAbs <= 3.141592){
                vThControl = 0.1;
              }
              else{
                vThControl = -0.1;
              }
            }
            else{
              if(deltaAngleAbs <= 3.141592){
                vThControl = -0.1;
              }
              else{
                vThControl = 0.1;
              }
            }

            uint64_t elapsedTime = micros();
            long long microseconds = elapsedTime - startTime;
            startTime = elapsedTime;
            std::cout << "GoFLM-" << yawRobot << "-" << angleRobotToLandmark << "-" << deltaAngleAbs << "-" << microseconds <<std::endl;
            //if(deltaAngleAbs < (1*3.141592/180)){
            if((deltaAngleAbs < (0.017))||((deltaAngle*deltaAnglePre) <= 0)){
            //if(false){
              vThControl = 0;
              gotoLandmarkFirstTimeStep = GO_TO_LANDMARK_FIRST_TIME_STEP_GOTO_LANDMARK;
              #ifdef DEBUG_CONTROL_TO_LANDMARK
                std::cout << "Go to landmark first time - rotate done"<<std::endl;
              #endif
            }
            else{
              deltaAnglePre = deltaAngle;
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
            
            double deltaAngleAbs = abs((double)(yawRobot - angleRobotToLandmark));
            if(deltaAngleAbs < (0.017)){
              vThControl = 0;
            }
            else if(yawRobot < angleRobotToLandmark){
              if(deltaAngleAbs <= 3.141592){
                vThControl = 0.1;
              }
              else{
                vThControl = -0.1;
              }
            }
            else{
              if(deltaAngleAbs <= 3.141592){
                vThControl = -0.1;
              }
              else{
                vThControl = 0.1;
              }
            }

            vxControl = 0.1;

            if(((xRobot >= (xToRun - 0.1))&&(xRobot <= (xToRun + 0.1)))&&((yRobot >= (yToRun - 0.1))&&(yRobot <= (yToRun + 0.1)))){
              vxControl = 0;
              vThControl = 0;
              #ifdef DEBUG_CONTROL_TO_LANDMARK
                std::cout << "Go to landmark first time - Robot move to point of landmark done"<<std::endl;
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
                std::cout << "Position of landmark no in any path"<<std::endl;
                gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
                break;
              }
              // swap
              x1 = x0;
              x0 = xToRun;
              double xVector = x1;
              y1 = y0;
              y0 = yToRun;
              double yVector = y1;

              if(y0 == yVector){
                if(x0 < xVector){
                  angleLandmark = 0;
                }
                else{
                  angleLandmark = 3.141592;
                }
              }
              else if(x0 == xVector){
                if(y0 < yVector){
                  angleLandmark = 1.570796;
                }
                else{
                  angleLandmark = 4.712388;
                }
              }
              else if((x0 < xVector)&&(y0 < yVector)){
                angleLandmark = atan((yVector - y0)/(xVector - x0));
              }
              else if((x0 > xVector)&&(y0 < yVector)){
                angleLandmark = atan(abs(yVector - y0)/(xVector - x0));
                angleLandmark = 3.141592 - angleLandmark;
              }
              else if((x0 > xVector)&&(y0 > yVector)){
                angleLandmark = atan(abs(yVector - y0)/abs(xVector - x0));
                angleLandmark = 3.141592 + angleLandmark;
              }
              else if((x0 < xVector)&&(y0 > yVector)){
                angleLandmark = atan(abs(yVector - y0)/abs(xVector - x0));
                angleLandmark = 6.283185 - angleLandmark;
              }

              deltaAngleAbs = abs((double)(yawRobot - angleLandmark));

              if(yawRobot < angleLandmark){
                if(deltaAngleAbs <= 3.141592){
                  vThControl = 0.1;
                }
                else{
                  vThControl = -0.1;
                }
              }
              else{
                if(deltaAngleAbs <= 3.141592){
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
            double deltaAngleAbs = abs((double)(yawRobot - angleLandmark));
            yawRobot = yawRobot + (vThControl*20/1000);

            if(yawRobot < angleLandmark){
              if(deltaAngleAbs <= 3.141592){
                vThControl = 0.1;
              }
              else{
                vThControl = -0.1;
              }
            }
            else{
              if(deltaAngleAbs <= 3.141592){
                vThControl = -0.1;
              }
              else{
                vThControl = 0.1;
              }
            }

            std::cout << "GoFLMA-" << yawRobot << "-" << angleLandmark << "-" << deltaAngleAbs << ""<<std::endl;

            if(deltaAngleAbs < (0.017)){
              vThControl = 0;
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
              indexLMCurrent = indexLMToRun;
              #ifdef DEBUG_CONTROL_TO_LANDMARK
                std::cout << "Go to landmark first time - go to landmark done"<<std::endl;
              #endif
            }

            break;
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        break;
      }
      case GO_TO_LANDMARK_PROCESS_STEP_RUN_TO_LANDMARK:{
        switch(gotoLandmarkStep){
          case GO_TO_LANDMARK_STEP_FIND_PATH_TO_GO:{
            uint8_t indexPathCurrent = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMCurrent]["i"];
            uint8_t indexPathToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["i"];
            uint8_t relationshipCount = pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["count"];
            uint8_t totalPath = pathAndLandmarkJson["pathAndLandmark"]["path"]["total"];

            //uint8_t pathAddingCount = 0;
            uint8_t arrPathIndexAdding[10], pathAddingCount = 0;
            uint8_t arrCheckRelationship[10][2];

            // L1 -> L2
            // L1 -> X1 -> L2
            // L1 -> X1 -> X2 -> L2
            // Loop for add sub path to go destination
            bool isFindedPathToGoDestination = false;
            while(pathAddingCount < (totalPath - 2)){
              if(pathAddingCount == 0){
                arrCheckRelationship[0][0] = indexPathCurrent;
                arrCheckRelationship[0][1] = indexPathToRun;

                bool isPathConnected = false;
                // With L1 -> X1
                // Check all relationship line
                for(uint8_t relationShipIndex = 0; relationShipIndex < relationshipCount; relationShipIndex++){
                  uint8_t countPathPresentOnRelationship = 0;
                  uint8_t numPathOnRelationshipLine = pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][0];
                  // With L1 -> X1
                  // With each line to check presence of L1 and X1
                  for(uint8_t k = 0; k < numPathOnRelationshipLine; k++){
                    if((pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][k + 1] == arrCheckRelationship[0][0])||
                      (pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][k + 1] == arrCheckRelationship[0][1])){
                      
                      countPathPresentOnRelationship++;
                    }
                  }
                  if(countPathPresentOnRelationship == 2){
                    isPathConnected = true;
                    break;
                  }
                }
                // If L1 -> X1 not connected -> Exit and adding sub path
                if(isPathConnected){
                  isFindedPathToGoDestination = true;
                  std::cout << "Finded path to go destination"<<std::endl;
                  break;
                }
                else{
                  pathAddingCount++;
                  std::cout << "Path adding: " << std::to_string(pathAddingCount) << ""<<std::endl;
                }
              }
              else{
                // Init array path adding
                for(uint8_t i = 0; i < pathAddingCount; i++){
                  arrPathIndexAdding[i] = 0;
                }

                bool isRunning_t = true;
                while(isRunning_t){
                  // Check for the same two path
                  bool twoPathIsSame = false;
                  uint8_t arrPathToCheckSame[16];
                  arrPathToCheckSame[0] = indexPathCurrent;
                  for(uint8_t i = 0; i < pathAddingCount; i++){
                    arrPathToCheckSame[i + 1] = arrPathIndexAdding[i];
                  }
                  arrPathToCheckSame[pathAddingCount + 1] = indexPathToRun;
                  for(uint8_t i = 0; i < (pathAddingCount + 2 - 1); i++){
                    for(uint8_t j = (i + 1); j < (pathAddingCount + 2); j++){
                      if(arrPathToCheckSame[i] == arrPathToCheckSame[j]){
                        twoPathIsSame = true;
                        break;
                      }
                    }
                    if(twoPathIsSame) break;
                  }

                  if(twoPathIsSame){
                    for(uint8_t i = (pathAddingCount - 1); i >= 0; i--){
                      arrPathIndexAdding[i]++;
                      if(arrPathIndexAdding[i] == totalPath){
                        arrPathIndexAdding[i] = 0;

                        if(pathAddingCount == (totalPath - 1)){
                          if(i == 0){
                            isRunning_t = false;
                            break;
                          }
                        }
                        else if(i == 0){
                          pathAddingCount++;
                          for(uint8_t i = 0; i < pathAddingCount; i++){
                            arrPathIndexAdding[i] = 0;
                          }
                        }
                      }
                      else{
                        break;
                      }
                    }
                  }
                  else{
                    arrCheckRelationship[0][0] = indexPathCurrent;
                    arrCheckRelationship[0][1] = arrPathIndexAdding[0];

                    //std::cout << "Path check: " << std::to_string(arrCheckRelationship[0][0]) << "->" << std::to_string(arrCheckRelationship[0][1]) + "->";

                    for(uint8_t i = 0; i < (pathAddingCount - 1); i++){
                      arrCheckRelationship[i + 1][0] = arrPathIndexAdding[i];
                      arrCheckRelationship[i + 1][1] = arrPathIndexAdding[i + 1];

                      //std::cout << std::to_string(arrCheckRelationship[i + 1][1]) + "->";
                    }

                    arrCheckRelationship[pathAddingCount][0] = arrPathIndexAdding[pathAddingCount - 1];
                    arrCheckRelationship[pathAddingCount][1] = indexPathToRun;
                    //std::cout << std::to_string(arrCheckRelationship[pathAddingCount][1]) << ""<<std::endl;

                    uint8_t arrCheckRelationshipCount = pathAddingCount + 1;
                    // L1 -> X1 -> L2
                    // Loop for check L1 -> X1 and after check X1 -> L2
                    bool isNeedAddSubPath = false;
                    for(uint8_t indexPathToPath = 0; indexPathToPath < (pathAddingCount + 1); indexPathToPath++){
                      bool isPathConnected = false;
                      // With L1 -> X1
                      // Check all relationship line
                      for(uint8_t relationShipIndex = 0; relationShipIndex < relationshipCount; relationShipIndex++){
                        uint8_t countPathPresentOnRelationship = 0;
                        uint8_t numPathOnRelationshipLine = pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][0];
                        // With L1 -> X1
                        // With each line to check presence of L1 and X1
                        for(uint8_t k = 0; k < numPathOnRelationshipLine; k++){
                          if((pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][k + 1] == arrCheckRelationship[indexPathToPath][0])||
                            (pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["dat"][relationShipIndex][k + 1] == arrCheckRelationship[indexPathToPath][1])){
                            
                            countPathPresentOnRelationship++;
                          }
                        }
                        if(countPathPresentOnRelationship == 2){
                          isPathConnected = true;
                          break;
                        }
                      }
                      // If L1 -> X1 not connected -> Exit and adding sub path
                      if(!isPathConnected){
                        isNeedAddSubPath = true;
                        break;
                      }
                    }

                    if(!isNeedAddSubPath){
                      isFindedPathToGoDestination = true;
                      std::cout << "Finded path to go destination"<<std::endl;
                      break;
                    }

                    //pathAddingCount++;
                    //std::cout << "Path adding: " << std::to_string(pathAddingCount) << ""<<std::endl;
                    for(uint8_t i = (pathAddingCount - 1); i >= 0; i--){
                      arrPathIndexAdding[i]++;
                      if(arrPathIndexAdding[i] == totalPath){
                        arrPathIndexAdding[i] = 0;

                        if(pathAddingCount == (totalPath - 1)){
                          if(i == 0){
                            isRunning_t = false;
                            break;
                          }
                        }
                        else if(i == 0){
                          pathAddingCount++;
                          for(uint8_t i = 0; i < pathAddingCount; i++){
                            arrPathIndexAdding[i] = 0;
                          }
                        }
                      }
                      else{
                        break;
                      }
                    }
                  }
                }

                break;
              }
            }
            
            if(isFindedPathToGoDestination){
              std::cout << "Get path done, go to set point array on path"<<std::endl;
              pathIdArrRunningLength = pathAddingCount + 2;
              pathIdArrRunning[0] = indexPathCurrent;
              for(uint8_t i = 0; i < pathAddingCount; i++){
                pathIdArrRunning[i + 1] = arrPathIndexAdding[i];
              }
              pathIdArrRunning[pathAddingCount + 1] = indexPathToRun;

              indexPathIdRunning = 0;
              gotoLandmarkStep = GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH;
            }
            else{
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
              std::cout<<"Not finded path to go destination"<<std::endl;
            }
            break;
          }
          case GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH:{
            uint8_t pointSize = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][0][0];
            
            // Convert xToRun and yToRun to mmm
            double temp = xToRun*1000;
            int32_t xToRun_t = temp;
            temp = yToRun*1000;
            int32_t yToRun_t = temp;

            std::cout << "Getpoint-Robot Position: " << std::to_string(xToRun) << std::to_string(yToRun) << ""<<std::endl;
            if((xToRun_t == pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1][0])&&
               (yToRun_t == pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1][1])){
              for(uint8_t i = 0; i < pointSize; i++){
                pointArrRunning[i][0] = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + i][0];
                pointArrRunning[i][1] = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + i][1];
              }
              pointArrRunningLength = pointSize;
            }
            else if((xToRun_t == pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + pointSize - 1][0])&&
                    (yToRun_t == pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + pointSize - 1][1])){
              for(uint8_t i = 0; i < pointSize; i++){
                pointArrRunning[i][0] = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + pointSize - i - 1][0];
                pointArrRunning[i][1] = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["a"][1 + pointSize - i - 1][1];
              }
              pointArrRunningLength = pointSize;
            }
            else{
              std::cout << "Not find point array to running"<<std::endl;
              vxControl = 0;
              vThControl = 0;
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
              break;
            }
            
            isNewReceivedRobotData = true;    // Set variable to calculate the angle robot to the point
            // Assign new point to running
            indexPointRunning = 0;
            std::cout << "Getpoint-Point array running: " << std::to_string(pointArrRunning[indexPointRunning][0]) << "-" << std::to_string(pointArrRunning[indexPointRunning][1]) << ""<<std::endl;
            xToRun = double(pointArrRunning[indexPointRunning][0]/1000);
            yToRun = double(pointArrRunning[indexPointRunning][1]/1000);
            std::cout << "Getpoint-Position to running: " << std::to_string(xToRun) << "-" << std::to_string(yToRun) << ""<<std::endl;

            // Based on path type to set different velocity
            std::string pathType = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][pathIdArrRunning[indexPathIdRunning]]["n"];
            if(pathType.at(0) == 'L'){
              vxSet = 0.1;
            }
            else if(pathType.at(0) == 'B'){
              vxSet = 0.05;
            }
            else{
              // Error
            }
  
            std::cout << "Get point array done, go to running..."<<std::endl;
            isGetFirstDeltaAngle = true;
            gotoLandmarkStep = GO_TO_LANDMARK_STEP_RUNNING_PROCESS;
            break;
          }
          case GO_TO_LANDMARK_STEP_RUNNING_PROCESS:{
            if(isNewReceivedRobotData){
              isNewReceivedRobotData = false;
              if(yRobot == yToRun){
                if(xRobot < xToRun){
                  angleRobotToPoint = 0;
                }
                else{
                  angleRobotToPoint = 3.141592;
                }
                std::cout<<"0"<<std::endl;
              }
              else if(xRobot == xToRun){
                if(yRobot < yToRun){
                  angleRobotToPoint = 1.570796;
                }
                else{
                  angleRobotToPoint = 4.712388;
                }
                std::cout<<"1"<<std::endl;
              }
              else if((xRobot < xToRun)&&(yRobot < yToRun)){
                angleRobotToPoint = atan(abs((yToRun - yRobot)/(xToRun - xRobot)));
                std::cout<<"2"<<std::endl;
              }
              else if((xRobot > xToRun)&&(yRobot < yToRun)){
                angleRobotToPoint = atan(abs((yToRun - yRobot)/(xToRun - xRobot)));
                angleRobotToPoint = 3.141592 - angleRobotToPoint;
                std::cout<<"3"<<std::endl;
              }
              else if((xRobot > xToRun)&&(yRobot > yToRun)){
                angleRobotToPoint = atan(abs((yToRun - yRobot)/abs(xToRun - xRobot)));
                angleRobotToPoint = 3.141592 + angleRobotToPoint;
                std::cout<<"4"<<std::endl;
              }
              else if((xRobot < xToRun)&&(yRobot > yToRun)){
                angleRobotToPoint = atan(abs((yToRun - yRobot)/abs(xToRun - xRobot)));
                angleRobotToPoint = 6.283185 - angleRobotToPoint;
                std::cout<<"5"<<std::endl;
              }
            }
            
            double deltaAngleAbs = abs((double)(yawRobot - angleRobotToPoint));
            double deltaAngle = ((double)(yawRobot - angleRobotToPoint));
            if(isGetFirstDeltaAngle){
              isGetFirstDeltaAngle = false;
              deltaAnglePre = deltaAngle;
            }
            yawRobot = yawRobot + (vThControl*20/1000);
            double deltaAngle_t;
            if(deltaAngleAbs <= 3.14159265){
              deltaAngle_t = deltaAngleAbs;
            }
            else{
              deltaAngle_t = (3.14159265*2) - deltaAngleAbs;
            }
            if(deltaAngle_t <= DegToRad(10)){
              vThSet = 0.05;
            }
            else if(deltaAngle_t <= DegToRad(20)){// 0.04
              vThSet = 0.08;
            }
            else if(deltaAngle_t <= DegToRad(30)){// 0.04
              vThSet = 0.12;
            }
            else if(deltaAngle_t <= DegToRad(40)){// 0.06
              vThSet = 0.18;
            }
            else if(deltaAngle_t <= DegToRad(50)){// 0.07
              vThSet = 0.25;
            }
            else if(deltaAngle_t <= DegToRad(60)){// 0.08
              vThSet = 0.32;
            }
            else if(deltaAngle_t <= DegToRad(70)){// 0.09
              vThSet = 0.40;
            }
            else if(deltaAngle_t <= DegToRad(80)){// 0.1
              vThSet = 0.5;
            }
            else{
              vThSet = 0.6;
            }
            if(deltaAngle_t <= DegToRad(5)){
              vThSet = deltaAngle_t*1/1.3962;
            }
            else if(deltaAngle_t <= DegToRad(15)){
              vThSet = deltaAngle_t*1.2/1.3962;
            }
            else if(deltaAngle_t <= DegToRad(30)){
              vThSet = deltaAngle_t*1.6/1.3962;
            }
            else if(deltaAngle_t <= DegToRad(45)){
              vThSet = deltaAngle_t*1.8/1.3962;
            }
            else if(deltaAngle_t <= DegToRad(80)){
              vThSet = deltaAngle_t*2/1.3962;
            }
            else{
              vThSet = 2.5;
            }
            std::cout<<"Delta angle: " << std::to_string(deltaAngleAbs) << " - " << std::to_string(deltaAngle_t) << " - "<< std::to_string(vThSet) << " - "<< std::to_string(deltaAngle) << " - "<< std::to_string(deltaAnglePre) << " - " << std::to_string(angleRobotToPoint) << std::endl;
            //std::cout<<"vThSet: " << std::to_string(vThSet) << ""<<std::endl;
            //vThSet = 0.1;
            if((deltaAngleAbs < (0.017))||((deltaAngle*deltaAnglePre) <= 0)){
              vThControl = 0;
            }
            else if(yawRobot < angleRobotToPoint){
              if(deltaAngleAbs <= 3.141592){
                vThControl = vThSet;
              }
              else{
                vThControl = -vThSet;
              }
            }
            else{
              if(deltaAngleAbs <= 3.141592){
                vThControl = -vThSet;
              }
              else{
                vThControl = vThSet;
              }
            }
            deltaAnglePre = deltaAngle;

            vxControl = 0.08;

            std::cout << "Pos: " << std::to_string(xRobot) << "-" << std::to_string(yRobot) << "-" << std::to_string(xToRun) << "-" << std::to_string(yToRun) << std::endl;
            if(((xRobot >= (xToRun - 0.05))&&(xRobot <= (xToRun + 0.05)))&&((yRobot >= (yToRun - 0.05))&&(yRobot <= (yToRun + 0.05)))){
              #ifdef DEBUG_CONTROL_TO_LANDMARK
                std::cout << "Running to point finish, index point: " << std::to_string(indexPointRunning) <<""<<std::endl;
              #endif

              indexPointRunning++;
              if(indexPointRunning >= pointArrRunningLength){
                indexPointRunning = 0;
                indexPathIdRunning++;

                vxControl = 0;
                vThControl = 0;
                if(indexPathIdRunning >= pathIdArrRunningLength){
                  std::cout << "Go to landmark done!"<<std::endl;
                  gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
                }
                else{
                  gotoLandmarkStep = GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH;
                }
              }
              else{
                xToRun = pointArrRunning[indexPointRunning][0]/1000;
                yToRun = pointArrRunning[indexPointRunning][1]/1000;

                std::cout << "Running-Position to running: " << std::to_string(xToRun) << std::to_string(yToRun) << ""<<std::endl;
              }
            }
            break;
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        break;
      }
    }
  }
}

void SerialCommunicate(){
	using LibSerial::SerialPort;
	using LibSerial::SerialStream;

	// You can instantiate a Serial Port or a Serial Stream object, whichever you'd prefer to work with.
	// For this example, we will demonstrate by using both types of objects.
	

	// Open hardware serial ports using the Open() method.
  bool connectedSerial = false;
  while (!connectedSerial)
  {
    try{
      serial_port.Open("/dev/ttyUSB0");
      connectedSerial = true;
    }
    catch(const OpenFailed&){
      connectedSerial = false;
    }
  }

	// Set the baud rates.
	using LibSerial::BaudRate;
	serial_port.SetBaudRate(BaudRate::BAUD_115200);


	// Prepare data to send
	const int BUFFER_SIZE = 16;
	unsigned char dataSend[BUFFER_SIZE];

	while(true){
    uint32_t velocityLeft = 0, velocityRight = 0;			// milimeters/minute
    if(gotoLandmarkProcessStep == GO_TO_LANDMARK_PROCESS_STEP_IDLE){
      if(counterCheckWaitingReceivedFromTcp > 30){
        #ifdef DEBUG_TCP_COMMUNICATE
          std::cout << "\nTcp received slow";
        #endif
        controlDat = 0;
      }
      else{
        counterCheckWaitingReceivedFromTcp++;		// Cycle ~12ms
      }
      
      CalculateVelocityOfManualControl(controlDat, (uint32_t*)&velocityLeft, (uint32_t*)&velocityRight);
    }
    else{
      CalculateVelocityOfAutoControl((uint32_t*)&velocityLeft, (uint32_t*)&velocityRight);
    }

		DataBuffer writeBuffer;
		// Start byte
		writeBuffer.push_back(0x01);
		writeBuffer.push_back(0xff);
		// Left velocity
		writeBuffer.push_back((velocityLeft>>24)&0xff);
		writeBuffer.push_back((velocityLeft>>16)&0xff);
		writeBuffer.push_back((velocityLeft>>8)&0xff);
		writeBuffer.push_back((velocityLeft>>0)&0xff);
		// Right velocity
		writeBuffer.push_back((velocityRight>>24)&0xff);
		writeBuffer.push_back((velocityRight>>16)&0xff);
		writeBuffer.push_back((velocityRight>>8)&0xff);
		writeBuffer.push_back((velocityRight>>0)&0xff);
		// Byte reserved
		writeBuffer.push_back(0x00);
		writeBuffer.push_back(0x00);
		writeBuffer.push_back(0x00);
		writeBuffer.push_back(0x00);
		// Crc
		int crcValue = checkCRC(writeBuffer, 16);
#ifdef DEBUG_SERIAL_COMMUNICATE
		std::cout << "\nCrc Value: " + std::to_string(crcValue);
#endif
		writeBuffer.push_back((crcValue>>8)&0xff);
		writeBuffer.push_back((crcValue>>0)&0xff);

#ifdef DEBUG_SERIAL_COMMUNICATE
		std::cout << "\nFrame Send: ";
		for (size_t i = 0 ; i < writeBuffer.size() ; i++)
		{
			std::cout << "-" + std::to_string(writeBuffer.at(i)) << std::flush ;
		}
#endif

		auto timeStartSendMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
		//std::cout << "\nmilliseconds since epoch: " << millisec_since_epoch;

		serial_port.Write(writeBuffer);
		//serial_port.DrainWriteBuffer();


		// Wait for data to be available at the serial port.
		uint16_t timeoutMs = 0;
		bool isTimeout = false;
		while(!serial_port.IsDataAvailable()) 
		{
			usleep(1000);
			timeoutMs += 1;
			if(timeoutMs >= 500){
				isTimeout = true;
				break;
			}
		}

		if(!isTimeout){
			using LibSerial::ReadTimeout;
		
			DataBuffer read_buffer ;

			try
			{
				// Read as many bytes as are available during the timeout period.
				serial_port.Read(read_buffer, 16, 20) ;
			}
			catch (const ReadTimeout&)
			{
#ifdef DEBUG_SERIAL_COMMUNICATE
				std::cout << "\n";
				for (size_t i = 0 ; i < read_buffer.size() ; i++)
				{
						std::cout << std::to_string(read_buffer.at(i)) << std::flush ;
				}

				std::cerr << "\nThe Read() call timed out waiting for additional data.";
#endif
			}

#ifdef DEBUG_SERIAL_COMMUNICATE
			std::cout << "\nFrame Received: ";
			for (size_t i = 0 ; i < read_buffer.size() ; i++)
			{
				std::cout << "-" + std::to_string(read_buffer.at(i)) << std::flush ;
			}
#endif

      if(read_buffer.size() >= 16){
        uint32_t vLeft_t, vRight_t;
        vLeft_t = 0;
        vRight_t = 0;
        //vLeft_t |= ((read_buffer.at(2)<<24)&0xff000000);
        vLeft_t |= ((read_buffer.at(3)<<16)&0x00ff0000);
        vLeft_t |= ((read_buffer.at(4)<<8)&0x0000ff00);
        vLeft_t |= ((read_buffer.at(5)<<0)&0x000000ff);

        //vRight_t |= ((read_buffer.at(6)<<24)&0xff000000);
        vRight_t |= ((read_buffer.at(7)<<16)&0x00ff0000);
        vRight_t |= ((read_buffer.at(8)<<8)&0x0000ff00);
        vRight_t |= ((read_buffer.at(9)<<0)&0x000000ff);        

        if((vLeft_t < 2000)&&(vRight_t < 2000)){
          velocityRunningFloat[0] = vLeft_t;
          velocityRunningFloat[1] = vRight_t;
        }
        //std::cout << "\n---------" + std::to_string(read_buffer.at(2)) << std::flush ;
        if(read_buffer.at(2) == 0xff){
          //std::cout << "\n" + std::to_string(velocityRunningFloat[0])<< std::flush ;
          velocityRunningFloat[0] = velocityRunningFloat[0]*(-1);
          //std::cout << "\n" + std::to_string(velocityRunningFloat[0])<< std::flush ;
        }
        if(read_buffer.at(6) == 0xff){
          velocityRunningFloat[1] = velocityRunningFloat[1]*(-1);
        }
      }

      //std::cout << "\n" + std::to_string(velocityRunningFloat[0]) + "-" + std::to_string(velocityRunningFloat[1]) + "\n";
      
			uint32_t timeNowMs = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			
			uint32_t timeWork = timeNowMs - timeStartSendMs;

			if(timeWork < 20){
				uint32_t timeDelayUs = (CYCLE_TIME_SEND_MS - timeWork)*1000;
				usleep(timeDelayUs);
			}
		}
		else{
			// Write log
		}

    while(serial_port.IsDataAvailable()){
      DataBuffer read_buffer ;
      serial_port.Read(read_buffer, 1, 20) ;
    }

		usleep(10000);
	}
}

void CommunicateWithAppToControlManual(){
  bool isClientConnectControlManual = false;

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
  address.sin_port = htons( PORT_COMMUNICATE_WITH_APP_MANUAL_CONTROL );

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
    if(!isClientConnectControlManual){
      if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
                       (socklen_t*)&addrlen))<0)
      {
        perror("accept");
        exit(EXIT_FAILURE);
      }
      isClientConnectControlManual = true;
    }
    else{
      valread = read( new_socket , buffer, 1024);
      //printf("%s\n",buffer );
      //printf("%d\n",valread );
      #ifdef DEBUG_TCP_COMMUNICATE
				std::cout << "\nTcp received...";
			#endif

      if(valread == 0){
        isClientConnectControlManual = false;

      }
      else if(valread < 34){

      }
      else{
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
          errorReceivedManualControl++;
        }

        //counterReceivedManualControl++;
      }

			counterCheckWaitingReceivedFromTcp = 0;
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
    std::cout<<"TcpRunning..."<<std::endl;
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
    std::cout<<"\nlisten..."<<std::endl;
    perror("listen");
    exit(EXIT_FAILURE);
  }

  parameter.ServerFd = server_fd;
  return parameter;
}

void CalculateVelocityOfManualControl(uint8_t controlValue, uint32_t *leftValue, uint32_t *rightValue){
  if(controlValue == 0){
    *leftValue = 0;
    *rightValue = 0;
    counterReceivedNoConnect = 0;
  }
  else if(controlValue == 9){
    if(counterReceivedNoConnect >= 5){
      *leftValue = 0;
      *rightValue = 0;
    }
    else{
      counterReceivedNoConnect++;
    }
  }
  else{
    counterReceivedNoConnect = 0;
    switch(controlValue){
      case 1:{
        *leftValue = 4294330676;
        *rightValue = 4294330676;
        //*leftValue = 4294867296;
        //*rightValue = 4294867296;
        break;
      }
      case 2:{
        *leftValue = 4294861193;
        *rightValue = 106103;
        //*leftValue = 4294937296;
        //*rightValue = 30000;
        break;
      }
      case 3:{
        *leftValue = 636620;
        *rightValue = 636620;
        //*leftValue = 100000;
        //*rightValue = 100000;
        break;
      }
      case 4:{
        *leftValue = 106103;
        *rightValue = 4294861193;
        //*leftValue = 30000;
        //*rightValue = 4294937296;
        break;
      }
      case 5:{
        *leftValue = 0;
        *rightValue = 0;
        break;
      }
      case 6:{
        *leftValue = 0;
        *rightValue = 0;
        break;
      }
      case 7:{
        *leftValue = 0;
        *rightValue = 0;
        break;
      }
      case 8:{
        *leftValue = 0;
        *rightValue = 0;
        break;
      }
    }
  }
}

void CalculateVelocityOfAutoControl(uint32_t *leftValue, uint32_t *rightValue){
  double rightVelocity_t = (((2*vxControl) + (0.4*vThControl))/2);  // m/s
  double leftVelocity_t = (((2*vxControl) - (0.4*vThControl))/2);   // m/s

  double rightValueAbs_t = abs(rightVelocity_t*3183000*2);
  double leftValueAbs_t = abs(leftVelocity_t*3183000*2);
  if(rightVelocity_t < 0){
    uint32_t temp = rightValueAbs_t;
    temp = 4294967296 - temp;
    *rightValue = temp;
  }
  else{
    uint32_t temp = rightValueAbs_t;
    *rightValue = temp;
  }

  if(leftVelocity_t < 0){
    uint32_t temp = leftVelocity_t;
    temp = 4294967296 - temp;
    *leftValue = temp;
  }
  else{
    uint32_t temp = leftVelocity_t;
    *leftValue = temp;
  }
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
        //std::cout << buffer.data();
    }
    return result;
}

double DegToRad(double val){
  return ((val*(2*3.14159265))/360);
}

void ConfigLogFile(){
  time_t now = time(0);
  tm *ltm = localtime(&now);

  std::string strFile;
  std::string monthStr, dayStr, hourStr, minStr, secondsStr;
  if((1 + ltm->tm_mon) < 10){
    monthStr = "0" + std::to_string(1 + ltm->tm_mon);
  }
  else{
    monthStr = std::to_string(1 + ltm->tm_mon);
  }
  if(ltm->tm_mday < 10){
    dayStr = "0" + std::to_string(ltm->tm_mday);
  }
  else{
    dayStr = std::to_string(ltm->tm_mday);
  }
  if(ltm->tm_hour < 10){
    hourStr = "0" + std::to_string(ltm->tm_hour);
  }
  else{
    hourStr = std::to_string(ltm->tm_hour);
  }
  if(ltm->tm_min < 10){
    minStr = "0" + std::to_string(ltm->tm_min);
  }
  else{
    minStr = std::to_string(ltm->tm_min);
  }
  if(ltm->tm_sec < 10){
    secondsStr = "0" + std::to_string(ltm->tm_sec);
  }
  else{
    secondsStr = std::to_string(ltm->tm_sec);
  }

  strFile = "./Log/" + std::to_string(1900 + ltm->tm_year) + 
            monthStr + dayStr + hourStr + minStr + secondsStr + ".txt";

  std::freopen( strFile.c_str(), "w", stdout );
}

// Get time stamp in microseconds.
uint64_t micros(){
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}
