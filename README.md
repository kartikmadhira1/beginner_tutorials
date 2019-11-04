# ROS beginner tutorials - Publisher and Subscriber nodes

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Overview
This project provides steps to create a basic publisher and subscriber node in ROS. The process followed is as given in the official ROS tutorials [wiki](http://wiki.ros.org/ROS/Tutorials).

The nodes created are as follows:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)

## Dependencies
The following dependencies are required to run this package:

- ROS kinetic
- catkin
- Ubuntu 16.04

For installing ROS, follow the process given [here](http://wiki.ros.org/kinetic/Installation)
For installing catkin, follow the process given [here](http://wiki.ros.org/catkin#Installing_catkin)

**Note:** catkin is usually installed by default when ROS is installed.

## Build the package
To build the given project first create a catkin workspace, clone this repository and then build by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/kartikmadhira1/beginner_tutorials
cd ..
catkin_make
git checkout Week10_HW

```

## Running the package
Follow the given steps to run the project:

1. Open a new terminal and start roscore
```
roscore
```
2. Source the setup.bash file of your catkin_ws:
```
cd ~/catkin_ws/
source devel/setup.bash
```
3. Run the publisher node using rosrun 
```
rosrun beginner_tutorials talker
```
4. Run the subscriber node using rosrun 
```
rosrun beginner_tutorials listener
```


## Run the nodes using launch file
After building (catkin_make from catkin workspace), the launch file below can be used to run the listener and talker together:
```
roslaunch beginner_tutorials beginner_tutorials.launch rate:=20
```

## Modify string service
Once the nodes are up and running, a string can be modified using the `rosservice` calls.
Use `rosservice` command to change the string message as follows:
```
rosservice call /modify_string "This is a new modified string"
``` 
The talker now publishes the new modified string "This is a new modified string"
