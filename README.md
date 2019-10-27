# beginner_tutorials : First Publisher/Subscriber ROS package

## Overview
A new ROS package created as named beginner_tutorials. This has been achieved by following the [ROS Online Tutorials](http://wiki.ros.org/ROS/Tutorials/).
A publisher node "talker" and a subscriber node "listener" has been created. A string is published over the topic "chatter", and is being subcribed by the listener.

### Dependencies
```
ROS Kinetic
Ubuntu 16.04
```

### Assumptions
To run the package-
1) Above mentioned dependancies are available and running in the user's system.
2) catkin_ws workspace is properly setup.
(If your workspace is named something else, use that name in the below commands)

# How to build
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
