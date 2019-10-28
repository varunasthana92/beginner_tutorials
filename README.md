# First Publisher/Subscriber ROS package: beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview
A new ROS package created as named beginner_tutorials. This has been achieved by following the [ROS Online Tutorials](http://wiki.ros.org/ROS/Tutorials/).
A publisher node "talker" and a subscriber node "listener" have been created. A string is published over the topic "chatter", and is being subcribed to by the listener node.

### Dependencies
```
ROS Kinetic
Ubuntu 16.04 LTS
```

### Assumptions
To run the package-
1) Above mentioned dependancies are available and running in the user's system.
2) catkin_ws workspace is properly setup.
(If your workspace is named something else, use that name in the below commands)

### How to build and run
Download the package in the catkin_ws/src directory
```
$ cd catkin_ws
$ catkin_make
$ roscore
```

In a new terminal
```
$ rosrun beginner_tutorials talker
```

In a new terminal
```
$ rosrun beginner_tutorials listener
```
To exit use the command Ctrl + C in each ternimal.
