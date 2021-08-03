#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iterator>
#include <iostream>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int func(int *p)
{
    return (sizeof(p)/sizeof(*p));
}

int a[7] = {1,2,3,4,5,6,7};
unsigned char result[1000000] = {0};
void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->info.width);
  ROS_INFO("I heard: [%d]",msg->info.height);
  
  using namespace std;
  int arr[] = {2, 7, 1, 111};
  auto array_length = end(msg->data) - begin(msg->data);
  
  ROS_INFO("array_length: [%d]",array_length);
  unsigned long index;
  unsigned long max = 0;
  for(index = 0; index < array_length; index++){
    if((msg->data[index] > max)&&(msg->data[index] != (-1))){
      max = msg->data[index];
    }
  }

  unsigned char temp;
  unsigned long indexResult = 0;
  for(index = 0; index < (array_length - 8); index+=8){
    temp = 0;
    if(msg->data[index] > 30){
      temp |= 0x01;
    }
    if(msg->data[index + 1] > 30){
      temp |= 0x02;
    }
    if(msg->data[index + 2] > 30){
      temp |= 0x04;
    }
    if(msg->data[index + 3] > 30){
      temp |= 0x08;
    }
    if(msg->data[index + 4] > 30){
      temp |= 0x10;
    }
    if(msg->data[index + 5] > 30){
      temp |= 0x20;
    }
    if(msg->data[index + 6] > 30){
      temp |= 0x40;
    }
    if(msg->data[index + 7] > 30){
      temp |= 0x80;
    }

    result[indexResult] = temp;
    indexResult++;
  }

  ROS_INFO("max: [%d]",max);
}

void mapStore(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_INFO("I heard: [%d]",msg->info.width);
  ROS_INFO("I heard: [%d]",msg->info.height);
  
  using namespace std;
  int arr[] = {2, 7, 1, 111};
  auto array_length = end(msg->data) - begin(msg->data);
  
  ROS_INFO("array_length: [%d]",array_length);
  unsigned long index;
  unsigned long max = 0;
  for(index = 0; index < array_length; index++){
    if((msg->data[index] > max)&&(msg->data[index] != (-1))){
      max = msg->data[index];
    }
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber sub = n.subscribe("/map", 1000, chatterCallback);
  ros::Subscriber sub = n.subscribe("map", 5000, mapStore);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}