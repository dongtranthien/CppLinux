#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <cstdio>
#include <ctime>  
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define PORT_COMMUNICATE_WITH_MAIN_PROCESS              2008

int main(int argc, char** argv){
  auto end = std::chrono::system_clock::now();

  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "finished computation at " << std::ctime(&end_time) << "s\n";
  return 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  
  std::freopen( "output.txt", "w", stdout );
  std::freopen( "error.txt", "w", stderr );

  std::cout << "Output message" << std::endl;
  std::cerr << "Error message" << std::endl;
  return 0;
}

void CommunicateWithMainProcess(){
  std::string killPortStr = "sudo kill -9 `sudo lsof -t -i:";
  killPortStr = killPortStr + std::to_string(PORT_COMMUNICATE_WITH_MAIN_PROCESS) + "`";
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
  address.sin_port = htons( PORT_COMMUNICATE_WITH_MAIN_PROCESS );

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

    }
  }
}