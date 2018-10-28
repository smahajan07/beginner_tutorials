# Beginner Tutorials

## Overview
A simple example of publishing and subscribing to a topic and displaying the contents of the message received.
Here we will be publishing a custom message on the topic "chatter".
 
## Dependencies
* Ubuntu Xenial (16.04)
* ROS Kinetic

## Build Instructions
* If you already have a catkin workspace then:
```
cd <catkin workspace>
cd src
git clone git@github.com:smahajan07/beginner_tutorials.git
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
git clone git@github.com:smahajan07/beginner_tutorials.git
cd ..
catkin_make
```
## Run Instructions
Open three terminals and follow the steps:
(Make sure you are in the same catkin workspace as created/modified in the build section. 
1. Terminal 1
```
source devel/setup.bash
roscore
```
2. Terminal 2
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
3. Terminal 3
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
