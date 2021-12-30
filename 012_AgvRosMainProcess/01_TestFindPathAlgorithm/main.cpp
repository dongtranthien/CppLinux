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

#include "json.h"

using json = nlohmann::json;
json pathAndLandmarkJson;


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
bool isPathAndLandmarkJsonDataOk = false;
uint8_t indexLMCurrent = 255, indexLMToRun;
bool isReceivedLMToRun = false;
GotoLandmarkProcessStep gotoLandmarkProcessStep;
GotoLandmarkFirstTimeStep gotoLandmarkFirstTimeStep;
GotoLandmarkStep gotoLandmarkStep;
double xRobot, yRobot, yawRobot, vxControl = 0, vThControl = 0, xToRun, yToRun, angleRobotToLandmark, angleLandmark;
bool isNewReceivedRobotData = false;

void test();

int main(int argc, char **argv)
{
  std::string myText;
  std::ifstream MyReadFile("pathAndLandmark.txt");

  // Use a while loop together with the getline() function to read the file line by line
  uint8_t countRead = 0;
  while (getline (MyReadFile, myText)) {
    // Output the text from the file
    countRead++;
  }
  std::cout<<countRead<<"\n";

  try{
    pathAndLandmarkJson = json::parse(myText);
  }
  catch (json::parse_error& ex){
    std::cerr << "JSON parse error at byte " << ex.byte << std::endl;
  }

  // Close the file
  MyReadFile.close();

  indexLMCurrent = 0;
  indexLMToRun = 1;

  std::string a = pathAndLandmarkJson["pathAndLandmark"]["path"]["dat"][0]["n"];
  if(a.at(0) == 'L'){
    std::cout<<"Done\n";
  }

  //test();

  return 0;
}

void test(){
  switch(gotoLandmarkStep){
          case GO_TO_LANDMARK_STEP_FIND_PATH_TO_GO:{
            uint8_t indexPathCurrent = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMCurrent]["i"];
            uint8_t indexPathToRun = pathAndLandmarkJson["pathAndLandmark"]["landmark"]["dat"][indexLMToRun]["i"];
            uint8_t relationshipCount = pathAndLandmarkJson["pathAndLandmark"]["path"]["relationship"]["count"];
            uint8_t totalPath = pathAndLandmarkJson["pathAndLandmark"]["path"]["total"];

            //uint8_t pathAddingCount = 0;
            uint8_t arrPathAdding[10], pathAddingCount = 0;
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
                  std::cout << "Finded path to go destination\n";
                  break;
                }
                else{
                  pathAddingCount++;
                  std::cout << "Path adding: " << std::to_string(pathAddingCount) << "\n";
                }
              }
              else{
                // Init array path adding
                for(uint8_t i = 0; i < pathAddingCount; i++){
                  arrPathAdding[i] = 0;
                }

                bool isRunning_t = true;
                while(isRunning_t){
                  // Check for the same two path
                  bool twoPathIsSame = false;
                  uint8_t arrPathToCheckSame[16];
                  arrPathToCheckSame[0] = indexPathCurrent;
                  for(uint8_t i = 0; i < pathAddingCount; i++){
                    arrPathToCheckSame[i + 1] = arrPathAdding[i];
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
                      arrPathAdding[i]++;
                      if(arrPathAdding[i] == totalPath){
                        arrPathAdding[i] = 0;

                        if(pathAddingCount == (totalPath - 1)){
                          if(i == 0){
                            isRunning_t = false;
                            break;
                          }
                        }
                        else if(i == 0){
                          pathAddingCount++;
                          for(uint8_t i = 0; i < pathAddingCount; i++){
                            arrPathAdding[i] = 0;
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
                    arrCheckRelationship[0][1] = arrPathAdding[0];

                    //std::cout << "Path check: " << std::to_string(arrCheckRelationship[0][0]) << "->" << std::to_string(arrCheckRelationship[0][1]) + "->";

                    for(uint8_t i = 0; i < (pathAddingCount - 1); i++){
                      arrCheckRelationship[i + 1][0] = arrPathAdding[i];
                      arrCheckRelationship[i + 1][1] = arrPathAdding[i + 1];

                      //std::cout << std::to_string(arrCheckRelationship[i + 1][1]) + "->";
                    }

                    arrCheckRelationship[pathAddingCount][0] = arrPathAdding[pathAddingCount - 1];
                    arrCheckRelationship[pathAddingCount][1] = indexPathToRun;
                    //std::cout << std::to_string(arrCheckRelationship[pathAddingCount][1]) << "\n";

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
                      std::cout << "Finded path to go destination\n";
                      break;
                    }

                    //pathAddingCount++;
                    //std::cout << "Path adding: " << std::to_string(pathAddingCount) << "\n";
                    for(uint8_t i = (pathAddingCount - 1); i >= 0; i--){
                      arrPathAdding[i]++;
                      if(arrPathAdding[i] == totalPath){
                        arrPathAdding[i] = 0;

                        if(pathAddingCount == (totalPath - 1)){
                          if(i == 0){
                            isRunning_t = false;
                            break;
                          }
                        }
                        else if(i == 0){
                          pathAddingCount++;
                          for(uint8_t i = 0; i < pathAddingCount; i++){
                            arrPathAdding[i] = 0;
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
              std::cout << "Get path done, go to set point array on path\n";
              gotoLandmarkStep = GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH;
            }
            else{
              gotoLandmarkProcessStep = GO_TO_LANDMARK_PROCESS_STEP_IDLE;
              std::cout<<"Not finded path to go destination\n";
            }
            break;
          }
          case GO_TO_LANDMARK_STEP_GET_POINT_ARRAY_ON_PATH:{
            break;
          }
          case GO_TO_LANDMARK_STEP_RUNNING_PROCESS:{
            break;
          }
        }
}
