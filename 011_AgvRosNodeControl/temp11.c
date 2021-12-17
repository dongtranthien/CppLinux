#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "crc.h"

#define DEBUG_TCP_COMMUNICATE
#define DEBUG_SERIAL_COMMUNICATE
#define DEBUG_ROS
#define CYCLE_TIME_SEND_MS				20
#define DEBUG
#define PORT 2003

using namespace LibSerial;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

void CalculateVelocityControlDriver(uint8_t controlValue, uint32_t *leftValue, uint32_t *rightValue);

SerialPort serial_port;
uint32_t counterReceivedManualControl = 0, controlDat = 0, errorReceivedManualControl = 0;
uint32_t counterCheckWaitingReceivedFromTcp = 0;
int counterReceivedNoConnect = 0;

void CheckExit(int s){
	if(s == 2){
		// Close the Serial Port and Serial Stream.
		serial_port.Close();
	}
}

void SerialCommunicate(){
	using LibSerial::SerialPort;
	using LibSerial::SerialStream;

	// You can instantiate a Serial Port or a Serial Stream object, whichever you'd prefer to work with.
	// For this example, we will demonstrate by using both types of objects.
	

	// Open hardware serial ports using the Open() method.
	serial_port.Open("/dev/ttyUSB0");

	// Set the baud rates.
	using LibSerial::BaudRate;
	serial_port.SetBaudRate(BaudRate::BAUD_115200);


	// Prepare data to send
	const int BUFFER_SIZE = 16;
	unsigned char dataSend[BUFFER_SIZE];

	while(true){
		if(counterCheckWaitingReceivedFromTcp > 40){
			#ifdef DEBUG_TCP_COMMUNICATE
				std::cout << "\nTcp received slow";
			#endif
			controlDat = 0;
		}
		else{
			counterCheckWaitingReceivedFromTcp++;		// Cycle ~12ms
		}
		uint32_t velocityLeft = 0, velocityRight = 0;			// milimeters/minute
		CalculateVelocityControlDriver(controlDat, (uint32_t*)&velocityLeft, (uint32_t*)&velocityRight);

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
		std::cout << "\nCrc Value: " + crcValue;
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

		usleep(10000);
	}
}

void RosRunning(){
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = -0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);

	while(1);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
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
    r.sleep();
  }
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
				errorReceivedManualControl++;
      }

      counterReceivedManualControl++;

      if(valread == 0){
        isClientConnect = false;
      }

			counterCheckWaitingReceivedFromTcp = 0;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AgvRosNodeControlMain");

  std::thread t1(SerialCommunicate);
  //std::thread t2(RosRunning);
	//std::thread t3(TcpRunning);
  t1.join();
  //t2.join();
	//t3.join();

  return 0;
}

void CalculateVelocityControlDriver(uint8_t controlValue, uint32_t *leftValue, uint32_t *rightValue){
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
        break;
      }
      case 2:{
        *leftValue = 4294861193;
        *rightValue = 106103;
        break;
      }
      case 3:{
        *leftValue = 636620;
        *rightValue = 636620;
        break;
      }
      case 4:{
        *leftValue = 106103;
        *rightValue = 4294861193;
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
