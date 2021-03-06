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
 *and the user can optionally modify the default message and
 *publish it at a different rate
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
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
  if (!ros::service::waitForService("custom_message_service")) {
    ROS_FATAL_STREAM("The custom message service isn't active. Try again");

    return 1;
  }
  // declare the tf broadcaster
  static tf::TransformBroadcaster br;
  // create a tf transform object
  tf::Transform transform;
  // input string for publishing
  std::string inpString;
  int pubRate;
  /**
  * Here we will check the number of arguments received and
  * will validate if we received the correct number of arguments.
  * One argument corresponds to the custom message and the other
  * for the rate of publishing
  */
  if (argc == 1) {
    ROS_WARN_STREAM(
        R"("No argument was passed, hence publishing the default string
            at the default publishing rate.")");
    inpString = "Hakuna_Matata!";
    pubRate = 10;
  } else if (argc > 1 && argc <= 3) {
    // if argument is passed that means base string needs to be modified
    ROS_DEBUG_STREAM("Received custom message and publishing rate correctly.");
    srv.request.inputString = argv[1];
    if (client.call(srv)) {
      inpString = srv.response.outputString;
    }
    pubRate = atoll(argv[2]);
    if (pubRate <= 0) {
      ROS_ERROR_STREAM("Publishing rate cannot be zero or negative!");

      return 1;
    }
  } else {
    // if multiple arguments were received, inform user of usage and exit
    ROS_FATAL_STREAM("Please provide appropriate arguments!");
    ROS_INFO_STREAM(
      R"("Usage: talker <custom message string> <rate of publishing>
        Note: If more than one word, please use _ (underscore) between words.
        OR No argument required for default string")");

    return 1;
  }
  ros::Rate loop_rate(pubRate);
  int count = 0;
  while (ros::ok()) {
    if (pubRate > 10) {
      ROS_WARN_STREAM(
        "Rate of publishing is high, other systems might be affected");
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
    // setting static values for origin and rotation
    transform.setOrigin(tf::Vector3(1.5, 1.5, 0.0));
    transform.setRotation(tf::Quaternion(1, 0.5, 1.5, 0.5));
    // broadcast the transform of child to parent
    br.sendTransform(tf::StampedTransform(transform, \
                    ros::Time::now(), "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
