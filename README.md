# beginner_tutorials
A beginner tutorials of ROS
---
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---

## Project Overview

This project implements the basics of the publisher subsciber model for ROS as given in the ROS WIKI page : http://wiki.ros.org/ROS/Tutorials

The model has two nodes:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)


## Dependencies
The project has following dependencies.

1. ROS Kinetic
2. catkin
3. Ubuntu 16.04 

- ROS INSTALLATION : http://wiki.ros.org/kinetic/Installation

- CATKIN INSTALLATION: http://wiki.ros.org/catkin#Installing_catkin (Usually installed by default when ROS is installed)

## Build steps
 To build the given project, create and build the catkin workspace by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
cd src/
git clone --recursive https://github.com/gautam-balachandran/beginner_tutorials.git
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
```

NOTE: For running command from each new terminal, source the devel/setup.bash file in the terminal before executing any ros command.

## Running the project

To run the publisher and subscriber model, follow the given steps:

1. First, start the ROSCORE
```
roscore
```

2. Keep the roscore running. Open a new terminal and type the following commands for the Talker node :
```
rosrun beginner_tutorials talker
```

3. To run Subscriber node, open a new terminal and type the following command :
```
rosrun beginner_tutorials listener
```

This will start roscore and talker and listener nodes in separate terminals.

## Running the project using the Launch file
After building the projec,run the talker and listener nodes using launch file as mentioned in the following steps :

1. Source the workspace as usual
```
source ~/catkin_ws/devel/setup.bash
```

2. Run the following command to launch using the launch file given
```
roslaunch beginner_tutorials stringChanger.launch
```

3. The publisher frequency is one of the input parameters that can be specified by user when runnning the launch file as shown below. If this frequency is not mentioned, the system will take the default frequecy of 20.
```
roslaunch beginner_tutorials stringChanger.launch frequency:=50
```

## Running the service

1. Output_String is a custom service that has been added to the project. This can be used to modify the base string published by the talker. After building the project and launching the talker-listener nodes, the list of available services can be seen using the command :
```
roservice list
```

2. To run the service, enter the following command:
```
rosservice call /Output_String "This is an user entered string!"
```

This will update the base string published by the talker to "This is an user entered string!"

## Checking the log messages
The output of rqt_console with info and warn logger level messages has been added in the results directory of the repository. To run the GUI for checking logs run
```
rqt_console
```

## Inspecting TF frames
The talker node broadcasts the `/talk` child frame with static translation and rotation with `/world` parent frame. To inspect the TF frames, first run the talker and listener nodes using roslaunch as follows:
```
roslaunch beginner_tutorials stringChanger.launch
```

Now in new terminal, run the `tf_echo` command to verify the TF frames between talk and world as below:
```
rosrun tf tf_echo /world /talk
```

The output of this command is shown below:
```
At time 1573255738.581
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255739.180
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255740.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255741.180
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255742.180
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255743.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255744.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255745.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255746.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255747.180
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255748.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255749.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255750.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]
At time 1573255751.181
- Translation: [5.000, 5.000, 5.000]
- Rotation: in Quaternion [0.612, 0.354, -0.612, 0.354]
            in RPY (radian) [2.094, 1.571, 0.000]
            in RPY (degree) [120.000, 90.000, 0.000]

```

The visualization of TF framescan be seen using the command as shown below:
```
rosrun rqt_tf_tree rqt_tf_tree
```

To get a pdf output of this visualization, we use the following command:
```
rosrun tf view_frames
```

The generated pdf file shows the TF frame transmitted from parent to child frame. It is also added to the results folder.

## Running ROSTEST

Unit test cases for this project have been written using Google Test libraries and rostest framework of ROS. After writing the test cases, source the workspace. Then, run the test cases with the command :
```
catkin_make run_tests_beginner_tutorials
```

This will run the test suite and output the result on the terminal as follows:
```
[ROSUNIT] Outputting test results to /home/gautam/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTester.xml
[Testcase: testtalkerTester] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTester/serviceExistenceTest][passed]
[beginner_tutorials.rosunit-talkerTester/serviceExecutionTest][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/gautam/.ros/log/rostest-gautam-Inspiron-7373-10432.log
-- run_tests.py: verify result "/home/gautam/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTester.xml"
[100%] Built target run_tests_beginner_tutorials_rostest_test_talkerTester.launch
[100%] Built target run_tests_beginner_tutorials_rostest
[100%] Built target run_tests_beginner_tutorials
```
## Recording bag files
After making changes to the launch file to enable recording ROS messages, we set the parameter `record` to enable or disable recording. Then you can launch the nodes and start recording using the command :
```
roslaunch beginner_tutorials stringChanger.launch record:=true
```

This command will record the data published on the `/chatter` topic by the Talker and create a rosbag_result.bag file in results directory. This file name is mentioned in the launch file and can be edited by the user as needed.

## Examining and playing the recorded bag file
To examine the recorded rosbag file, run the following command:
```
rosbag info results/rosbag_result.bag
```

It will output the given info
```
path:        results/rosbag_result.bag
version:     2.0
duration:    30.9s
start:       Nov 08 2019 12:04:44.54 (1573232684.54)
end:         Nov 08 2019 12:05:15.41 (1573232715.41)
size:        384.7 KB
messages:    1845
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      307 msgs    : std_msgs/String
             /rosout       617 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   614 msgs    : rosgraph_msgs/Log 
             /tf           307 msgs    : tf2_msgs/TFMessage
```
To play the bag file, start roscore.
```
roscore
```
In another terminal, start just the listener, and not the talker.
```
rosrun beginner_tutorials listener
```

Now, in another terminal run `rosbag play` command as shown below:
```
rosbag play results/rosbag_result.bag
```
We can verify that the bag messages are received by the listener node by checking the listener node window. 
