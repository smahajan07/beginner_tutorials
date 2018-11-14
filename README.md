# Beginner Tutorials

## Overview
This respository is an adapted version of the basic ROS tutorials for publishing and subscribing to a topic and displaying the contents of the message received, creating a service to edit a string and publishing it, logging output, creating a launch file and taking in arguments, using the TF library in ROS, writing Unit Tests and recording and playing from a ROS bag. These are some commonly used features of ROS and even though a lot of resources are available online, it can sometimes still be tricky. Please refer to the official [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) for more details. They are a good source for all ROS beginner and even intermediate level users.  
 
## Dependencies
* Ubuntu Xenial (16.04)
* ROS Kinetic

## Build Instructions
* If you already have a catkin workspace then:
```
cd <catkin workspace>
cd src
git clone https://github.com/smahajan07/beginner_tutorials.git
cd ..
catkin_make
source devel/setup.bash
```

* If you do not have a catkin workspace, you can create one by following:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/smahajan07/beginner_tutorials.git
cd ..
catkin_make
```
## Rostest
Before you proceed to run it, check whether it passes the unit tests:
```
cd <your catkin workspace>
source devel/setup.bash
catkin_make run_tests
```
This should end with an output like
```
[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-testTalker/checkTF][passed]
[beginner_tutorials.rosunit-testTalker/checkCustomMsgServer][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```
## Run Instructions (Using Launch File)
```
cd <your catkin workspace>
source devel/setup.bash
# Option 1: Run with default message 
roslaunch beginner_tutorials allNodes.launch
```
```
# Option 2: Run with your custom message and publishing frequency
roslaunch beginner_tutorials allNodes.launch pubRate:=5 inpStr:=carpe_diem
# Note: pubRate:=<frequency of your choice> 
# inpStr:=<string_of_your_choice> // But currently it takes a single word or words connected with an underscore (_) 
```
* It will raise a fatal error if you enter a negative value or zero for frequency
* It will warn you that frequency is high if you enter a value more than 10
* To kill the terminal please press CTRL+C (twice, because additional output window is opened as well)

## Inspect TF frames
If your nodes are still running , then you can directly follow these steps ELSE please launch the nodes using the previous instructions. Open a new terminal and run:
```
rosrun tf tf_echo world talk
```
[Optional] If you wish to view them using view frames tool you can either check out the provided as ```results/frames.pdf``` OR you can run this command from the terminal and it will save the diagram in the directory from where the command was run. Again, make sure your nodes are running.
```
rosrun tf view_frames
```

## Rosbags

* Inspect

Along with the package a sample ROS bag was provided ```results/automatic.bag```. 
If you wish to look at the topics in the rosbag and find out more details about it:
```
cd <your catkin workspace>
source devel/setup.bash
cd src/beginner_tutorials/results
rosbag info automatic.bag
```
* Play

You can play the above mentioned rosbag by running the following in two different terminals:
```
roscore
```
 and
```
rosbag  play automatic.bag
```
* Record

If you wish to record the data in a rosbag you can run the rosbags command separately in a new terminal OR you can make use of the option provided in the launch file. You just need to set the record argument to true by running:
```
cd <your catkin workspace>
source devel/setup.bash
roslaunch beginner_tutorials allNodes.launch record:=true
```
This will save a rosbag in your results directory of the package, by the name ```automatic.bag```.

## Run Instructions (Calling the service from the terminal)
1. Terminal 1
```
roscore
```
2. Start the server
```
cd <your catkin workspace>
source devel/setup.bash
rosrun beginner_tutorials customMsgServer
```
3. Call the service
```
cd <your catkin workspace>
source devel/setup.bash
rosservice call /custom_message_service go_terps
# Usage: rosservice call /custom_message_service <word/string of your choice-connected with _>
```

## Run Instructions (Vanilla way)
Open four terminals and follow the steps: 
1. Terminal 1
```
roscore
```
2. Terminal 2
```
cd <your catkin workspace>
source devel/setup.bash
# Option 1: Run with default message 
roslaunch beginner_tutorials talker
```
```
# Option 2: Run with your custom message and publishing frequency
roslaunch beginner_tutorials talker carpe_diem 5
# Usage: roslaunch beginner_tutorials talker <string_of_your_choice> <frequency of your choice>
# For <string_of_your_choice> currently it takes a single word or words connected with an underscore (_)
```
3. Terminal 3
```
cd <your catkin workspace>
source devel/setup.bash
rosrun beginner_tutorials customMsgServer
rosrun beginner_tutorials listener
```
4. Terminal 4
```
cd <your catkin workspace>
source devel/setup.bash
rosrun beginner_tutorials listener
```
