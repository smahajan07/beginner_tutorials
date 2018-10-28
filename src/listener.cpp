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
 *@file listener.cpp
 *@author Sarthak Mahajan
 *@brief A simple node that subscribes to a topic and displays the contents
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 *@brief Call back for subscriber node. Displays contents of the message
 *@param msg is the input message which is received during callback
 *@return None
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  // print contents of received message
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  // initialise node
  ros::init(argc, argv, "listener");
  // create node handle
  ros::NodeHandle n;
  // create subscriber
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
