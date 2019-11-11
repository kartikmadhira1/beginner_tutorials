# ROS beginner tutorials

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Overview
This project provides steps to create a basic publisher and subscriber node in ROS. The code includes ROS transform(TF), Unit testing using `rostest` and `rosbag` recording and playback.The process followed is as given in the official ROS tutorials [wiki](http://wiki.ros.org/ROS/Tutorials).

The nodes created are as follows:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)
3. Service - modify_string service to modify the string published by talker.
4. TF broadcast - Broadcasts static transformation between /talk and /world frames.
5. test suite to test the existence and execution of modify_string service for talker node.

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
git checkout Week11_HW
cd ~/catkin_ws/
catkin_make
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

## Inspecting TF frames
The talker node broadcasts the static transformation between the /talk child frame and /world parent frame. Run the talker and listener nodes using roslaunch as follows:
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
Verify the tf frames between talk and world as below:
```
rosrun tf tf_echo /talk /world
```
Visualization of tf frames broadcasted can be viewed from the following command:
```
rosrun rqt_tf_tree rqt_tf_tree
```
A pdf file using the view_frames command as shown below:
```
rosrun tf view_frames
```

## Running tests with rostests: 

Unit test cases for the project have been written using gtest and rostest. To run the tests using catkin go to to catkin workspace root directory and issue the following command:
```
catkin_make run_tests_beginner_tutorials
```

## Recording bag files
To launch the nodes use the command:
```
roslaunch beginner_tutorials beginner_tutorials.launch record:=true
```
This command will record the data published on the /chatter topic by node /talker and create a listener.bag file in results directory.

## Examining and playing the recorded bag file
To examine the recorded `rosbag` file, run the following command:
```
rosbag info results/listener.bag
```

To play the `rosbag`, run `roscore` in one window and run the listener node:
```
roscore
rosrun beginner_tutorials listener
```
Now run `rosbag play` command to play the recorded messages:
```
rosbag play results/listener.bag
```
