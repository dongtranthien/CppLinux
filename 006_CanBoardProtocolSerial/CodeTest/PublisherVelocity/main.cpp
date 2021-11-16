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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo1");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("driverControl", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    std_msgs::String msg;

    std::stringstream ss;
    ss << "1155,2154";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}