# Beginner Tutorials

## Overview
A simple example of publishing and subscribing to a topic and displaying the contents of the message received.
Here we will be publishing a custom message on the topic "chatter".

[*New Feature*] Now the user can enter a custom string to publish and also choose the publishing rate.
 
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
rosservice call /custom_message_service go_terps!!
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
3. Terminal 4
```
cd <your catkin workspace>
source devel/setup.bash
rosrun beginner_tutorials listener
```
