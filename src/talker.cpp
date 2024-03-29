/*
 *  @file    talker.cpp
 *  @author  Gautam Balachandran
 *  @copyright MIT License
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @brief  Implements ROS publisher and subscriber nodes
 *
 *  @section DESCRIPTION
 *
 *  This module is part of the ROS beginner level tutorials.
 *  It defines the listener module of the program.
 *
 */

#define _USE_MATH_DEFINES

#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Output_String.h"

/**
 * Initializing the input string
 */

extern std::string stringMsg = "This is the custom input String!";

/**
 * @brief      changeOutputMessage function
 *
 * @param      req     request message
 * @param      res     response messsge
 *
 * @return     Boolean value representing callback success or failure
 */
bool changeOutputMessage(beginner_tutorials::Output_String::Request &req,
                         beginner_tutorials::Output_String::Response &res) {
  stringMsg = req.inputString;
  res.outputString = stringMsg;  // Output String modified

  // Warning message when the output message is modified
  ROS_WARN_STREAM("Output Message Modified!");

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Transform Broadcaster object
  static tf::TransformBroadcaster broadcast;
  tf::Transform transform;  // Transform Object

  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  ros::ServiceServer server = n.advertiseService("Output_String",
                                                 changeOutputMessage);

  ros::Rate loop_rate(10);
  int count = 0;

  if (!ros::ok()) {
    // Fatal message thrown when ROS node is not up
    ROS_FATAL_STREAM("ROS is not running!");
  }

  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    int inputFreq = 20;  // Setting default frequency

    if (argc == 2) {  // Checking if frequency is an argument
      inputFreq = atoi(argv[1]);
      // Debugging message to check the input frequency
      ROS_DEBUG_STREAM("Input frequency : " << inputFreq);

      if (inputFreq <= 0) {
        // Error message for invalid frequency
        ROS_ERROR_STREAM("Invalid publisher frequency");
      }
    } else {
      // Warning message when using default frequency
      ROS_WARN_STREAM("No input frequency, using default publisher frequency");
    }

    if (stringMsg == "") {
      // Debugging message when input string is empty
      ROS_DEBUG_STREAM("Input string is empty! Please add an input string");
      return 0;
    }

    ss << stringMsg << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    // Setting the translation and rotation
    transform.setOrigin(tf::Vector3(5.0, 5.0, 5.0));
    tf::Quaternion quart;
    // Setting Roll-Pitch-Yaw angles in the Quaternion
    quart.setRPY(M_PI, M_PI / 2, M_PI / 3);
    transform.setRotation(quart);

    /*Broadcast the transform with the world frame as parent and 
     the talk frame as the child.*/
    broadcast.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
