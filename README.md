# beginner_tutorials
A beginner tutorials of ROS

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


## Checking the log messages
The output of rqt_console with info and warn logger level messages has been added in the results directory of the repository. To run the GUI for checking logs run
```
rqt_console
```
