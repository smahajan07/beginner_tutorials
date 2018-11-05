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
 *@file talker.cpp
 *@author Sarthak Mahajan
 *@brief A simple node that publishes a custom message string
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "beginner_tutorials/CustomMsgSrv.h"

int main(int argc, char **argv) {
  // initialize node
  ros::init(argc, argv, "talker");
  // create node handle
  ros::NodeHandle n;
  // create publisher
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // create service client
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::CustomMsgSrv>
      ("custom_message_service");
  // create an object of the custom msg type
  beginner_tutorials::CustomMsgSrv srv;
  std::string inpString;
  int pubRate;
  if(argc == 1){
    ROS_WARN_STREAM(
        "No argument was passed, hence publishing the default string \
          at the default publishing rate.");
  }
  // if argument is passed that means base string needs to be modified
  else if(argc > 1 && argc <= 3){
    ROS_DEBUG_STREAM("Received custom message and publishing rate correctly.");
    srv.request.inputString = argv[1];
    if(client.call(srv)){
      inpString = srv.response.outputString;
    }
    pubRate = atoll(argv[2]);
    if(pubRate <= 0){
      ROS_ERROR_STREAM("Publishing rate cannot be zero or negative!");

      return 1;
    }
  }
  // if multiple arguments were received, inform user of usage and exit
  else if(argc > 3){
    ROS_FATAL_STREAM("Please provide appropriate arguments!");
    ROS_INFO_STREAM(
        "Usage: talker <custom message string> <rate of publishing>\n \
        Note: If more than one word, please use _ (underscore) between words.\n \
        OR No argument required for default string");

    return 1;
  }
  ros::Rate loop_rate(pubRate);
  int count = 0;
  while (ros::ok()) {
    if(pubRate > 10){
      ROS_WARN_STREAM("Rate of publishing is high, other systems \
        might be affected");
    }
    // create a message
    std_msgs::String msg;
    std::stringstream ss;
    ss << inpString << " " << count;
    msg.data = ss.str();
    // display message
    ROS_INFO_STREAM(msg.data);
    // publish message
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
