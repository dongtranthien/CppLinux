#include <iostream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    std::cout<<std::to_string(transform.getOrigin().x()) + "-" + std::to_string(transform.getOrigin().y()) + "-" + std::to_string(transform.getRotation().x()) + "-" + std::to_string(transform.getRotation().y()) + "-" + std::to_string(transform.getRotation().z()) + "-" + std::to_string(transform.getRotation().w()) + "\n";
    tf::Quaternion q(
        transform.getRotation().x(),
        transform.getRotation().y(),
        transform.getRotation().z(),
        transform.getRotation().w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = yaw*180/3.14159265;
    std::cout<<std::to_string(yaw) + "\n";

    rate.sleep();
  }
  return 0;
};