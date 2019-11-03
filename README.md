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
2. Keep the roscore running. Open a new terminal and type the following commands for the Talker node :-
```
rosrun beginner_tutorials talker
```

3. To run Subscriber node, open a new terminal and type the following command :-
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
