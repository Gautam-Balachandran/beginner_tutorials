/*
 *  @file    talkerTester.cpp
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
 *  @brief  Tester file for the Talker node
 *
 *  @section DESCRIPTION
 *
 *  This module is part of the ROS beginner level tutorials.
 *  It defines the tester module for the listener module of the program.
 *
 */


#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "beginner_tutorials/Output_String.h"


/**
 * @brief Test case that checks if the OutputString service exists.
 * @param none
 * @return none
 */
TEST(TESTSuite, serviceExistenceTest) {
    ros::NodeHandle rosNode;
    ros::ServiceClient client = rosNode.serviceClient<beginner_tutorials::
        Output_String>("Output_String");
    bool exists(client.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);
}

/**
 * @brief Test case that check the output of the Output_String service.
 * @param none
 * @return none
 */
TEST(TESTSuite, serviceExecutionTest) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::
        Output_String>("Output_String");
    beginner_tutorials::Output_String srv;
    srv.request.inputString = "Service Called!";
    client.call(srv);
    EXPECT_STREQ("Service Called!", srv.response.outputString.c_str());
}



/**
 * @brief Run all rostests for the talker node
 *
 * @param none
 * @return 0 for Success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "talkerTester");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
