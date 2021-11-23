#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <pthread.h>

#include "crc.h"

//#define DEBUG_SERIAL_COMMUNICATE
#define DEBUG_ROS
#define CYCLE_TIME_SEND_MS				20

using namespace LibSerial;

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

SerialPort serial_port;
uint32_t velocityMax = 15000;											// milimeters/minute
uint32_t velocityLeft = 0, velocityRight = 0;			// milimeters/minute

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
		//// Valid velocity value
		//if(velocityLeft >= velocityMax){
		//	// Write log
		//	velocityLeft = 0;
		//}
		//if(velocityRight >= velocityMax){
		//	// Write log
		//	velocityRight = 0;
		//}

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
	}
}

void receiveVelocity(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());

	std::string s = msg->data;
	std::string delimiter = ",";
	int indexDelimiter = s.find(delimiter);
	std::string velocityLeftStr = s.substr(0, indexDelimiter);
	std::string velocityRightStr = s.substr(indexDelimiter + 1, s.length());

	
	//std::cout<<token + "-" + s;
	velocityLeft = std::stoul(velocityLeftStr);
	velocityRight = std::stoul(velocityRightStr);

#ifdef DEBUG_ROS
	std::cout<<"Velocity Received From Topic driverControl: " + std::to_string(velocityLeft) + "-" + std::to_string(velocityRight) + "\n";
#endif
}

void RosRunning(){
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("driverControl", 5000, receiveVelocity);

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NodeDriverCanController");

  std::thread t1(SerialCommunicate);
  std::thread t2(RosRunning);
  t1.join();
  t2.join();

  return 0;
}
