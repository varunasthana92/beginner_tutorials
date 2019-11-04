# ROS package: beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview
A new ROS package created as named beginner_tutorials. This has been achieved by following the [ROS Online Tutorials](http://wiki.ros.org/ROS/Tutorials/).
A publisher node "talker" and a subscriber node "listener" have been created. A string is published over the topic "chatter", and is being subcribed to by the listener node. Talker node calls upon a service change_string_output, by acting as an client. Based on the response of the service, the published string toggles between "Varun" and "Asthana".

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

### How to build
Clone the package in the catkin_ws/src directory
```
$ cd catkin_ws/src
$ git clone https://github.com/varunasthana92/beginner_tutorials.git
$ cd ..
$ catkin_make
```
### How to run and interact with program
There are 2 ways to run the program-
1) Bring up each node individually via command line with required arguments, OR
2) Use a launch file (provided) in the package to take care of everything.

I will outline the steps for both methods.
#### Run without the launch file
```
$ roscore
```

In a new terminal
```
$ rosrun beginner_tutorials change_string_server
```

In a new terminal
```
$ rosparam set /talker/freq 10
```
Here the "freq" refers to the frequency of ros::Rate which controls the rate at which the data is published on the topic "chatter".
User can use any other positive value instead of 10.

```
$ rosrun beginner_tutorials talker
```

In a new terminal
```
$ rosrun beginner_tutorials listener
```
To exit use the command Ctrl + C in each ternimal, except for the terminal running roscore. This process can be terminated after executing the below command in a separate terminal-

Delete the paramter initially set by us. It can be done by
```
$ rosparam delete /talker/freq
```

#### Run with the launch file
In a new terminal
```
$ roslaunch beginner_tutorials beginner.launch
```
1 window each for the talker node and the listener node will open-up. The default frequency for data publishing is set at 10. But this can be overwritten by a command line argument as below. Do not forget to first end the above runninig roslaunch command with Ctrl + C.

```
$ roslaunch beginner_tutorials beginner.launch publish_frequency:=4
```
The argument publish_frequency is used to set the parameter value of /talker/freq. As before user may use any other positive value.

### How are we getting different string outputs? Lets examine our service change_string_output
A service server has been setup using the node change_string_server defined in ChangeStringServer.cpp. The request and response types for this service are defined in ChangeString.srv file, which is exactly same as below-
```
bool choice
---
string name
```
It takes a bool input request from the client and returns a string. The callback function for this service is defined in ChangeStringServer.cpp as:
```
bool changeOutput(beginner_tutorials::ChangeString::Request &req, beginner_tutorials::ChangeString::Response &resp) {
  /**
   * Based on the bool value passed by the client, an output string is selected
   */
  if (req.choice) {
    resp.name = "Varun";
  } else {
    resp.name = "Asthana";
  }
  return true;
}
```
In the talker node, the bool request is being toggled alternatively between TRUE and FALSE. Hence we were getting different output strings.

#### Lets try and run this service using the rosservice command line tool
In a new terminal
```
$ roscore
```

In a new terminal
```
$ rosrun beginner_tutorials change_string_server
```

In a new terminal
```
$ rosservice call /change_string_output true
```
Output is-
```
name: "Varun"
```
Retry with different request
```
$ rosservice call /change_string_output false
```
Output is-
```
name: "Asthana"
```
