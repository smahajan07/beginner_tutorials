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
 *@file customMsgServer.cpp
 *@author Sarthak Mahajan
 *@brief A simple service node that takes input from user and sends it to the
 *talker (publisher) node.
 */

#include "ros/ros.h"
#include "beginner_tutorials/CustomMsgSrv.h"

bool callbackCMS(beginner_tutorials::CustomMsgSrv::Request &req,
                 beginner_tutorials::CustomMsgSrv::Response &res) {
  // TODO(smahajan07): instead of returning the same string \
  add option to return upper case or camel case of the string
  res.outputString = req.inputString;
  ROS_INFO_STREAM("Sending response back");

  return true;
}

int main(int argc, char **argv) {
  // initialize node
  ros::init(argc, argv, "custom_message_server");
  // create node handle
  ros::NodeHandle n;
  // create service server
  ros::ServiceServer service = n.advertiseService("custom_message_service",
                                                  callbackCMS);
  ROS_INFO_STREAM("Ready to publish a custom message.");
  ros::spin();

  return 0;
}

