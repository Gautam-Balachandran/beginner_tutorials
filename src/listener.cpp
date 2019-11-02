 /*
 *  @file    listener.cpp
 *  @author  Gautam Balachandran
 *  @copyright MIT License
 *
 *  @brief  Implements ROS publisher and subscriber nodes
 *
 *  @section DESCRIPTION
 *
 *  This module is part of the ROS beginner level tutorials.
 *  It defines the listener module of the program.
 *
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/OutputString.h"

void chatterCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Incoming Message: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
