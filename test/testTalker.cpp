/** MIT License
 Copyright (c) 2018 Sarthak Mahajan
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/**
 *@copyright Copyright (c) 2018 Sarthak Mahajan
 *@file test.cpp
 *@author Sarthak Mahajan
 *@brief All the test cases are defined here 
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/CustomMsgSrv.h"

TEST(TESTTalker, checkTF) {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("world", "talk", ros::Time(0),
                            ros::Duration(10.0));
  listener.lookupTransform("world", "talk", ros::Time(0), transform);
  EXPECT_EQ(0.0, transform.getOrigin().z());
}

TEST(TESTTalker, checkCustomMsgServer) {
  ros::NodeHandle n;
  ros::service::waitForService("custom_message_service");
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::CustomMsgSrv>
      ("custom_message_service");
  beginner_tutorials::CustomMsgSrv srv;
  std::string inpString = "Gaussian";
  srv.request.inputString = inpString;
  client.call(srv);
  std::string expectedString = inpString + "| What a choice of string!";
  EXPECT_EQ(expectedString, srv.response.outputString);
}
