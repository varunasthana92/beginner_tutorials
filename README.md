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
(If your workspace is named something else or is at other path, use that name and/or path in the below commands)

### How to build the program
Clone the package in the catkin_ws/src directory
```
$ cd catkin_ws/src
$ git clone https://github.com/varunasthana92/beginner_tutorials.git
$ cd ..
$ catkin_make
```
### How to build and run test
```
$ cd catkin_ws/
$ catkin_make run_tests
```
### How to check cpplint and cppcheck
Use the below commands to run cppcheck and cpplint.
```
$ cd ~/catkin_ws/src/beginner_tutorials
$ cppcheck $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./test/" -e "^./docs/" -e "^./results" )
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./test/" -e "^./docs/" -e "^./results" )
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

#### Data recording by rosbag
The launch file also has a tag to initiate data recording of all the topics in a bag file "BagData". By default this functionality is kept"ON", which will save a new .bag file (or replace any existing file with same name) in /results directory, eaach tiime the launch file is executed. User has the option to disable the recording of data by passing an argument set to 0 as below while running the launch file.

```
$ roslaunch beginner_tutorials beginner.launch record_data:=0
```
Note: If user wants, he may pass both arguments in a single line command.
```
$ roslaunch beginner_tutorials beginner.launch record_data:=0 publish_frequency:=4
```
#### Inspecting TF Frame
Ensure that the program is running either by launch command or manually as instructed above. A "talk" frame is broadcasted with "world" frame as its parent. rqt tree of the tf frames can be viewed in an interactive GUI with below command in a new terminal. 
```
$ rosrun rqt_tf_tree rqt_tf_tree
```
A pdf file for the same can be generated with below commands-
```
$ rosrun tf view_frames
```
To view the pdf-
```
$ evince frames.pdf
```
To see the data of the broadcasted frame, use the below command
```
$ rosrun tf tf_echo world talk
```
#### Inspecting and playing back the rosbag file
Terminate all instances of the program and ros. Use the below set of commands to inspect the previously created BagData.bag file and playback the same with listener node without running the talker node.

In a new terminal
```
$ roscore
```
In a new terminal (If your catkin_ws is located at some other path, use that path)
```
$ cd ~/catkin_ws
$ cd /src/beginner_tutorials/results
$ rosbag info BagData.bag
```
output will be something similar to-
```
path:        BagData.bag
version:     2.0
duration:    5.2s
start:       Nov 10 2019 20:57:39.62 (1573437459.62)
end:         Nov 10 2019 20:57:44.80 (1573437464.80)
size:        73.2 KB
messages:    288
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter       41 msgs    : std_msgs/String
             /rosout       104 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   102 msgs    : rosgraph_msgs/Log 
             /tf            41 msgs    : tf2_msgs/TFMessage
```
Keep this terminal open, and we will come back to it.

In a new terminal
```
$ rosrun beginner_tutorials listener
```
Nothing will be displayed at the moment except a blinking cursor. This will change once we playback the data from our bag file.

To play this bag file, use the below command.
Use the same terminal in which we ran the rosbag info command, as we need to be in the directory where the file is available.
```
$ rosbag play BagData.bag
```
You can now see that the listener node is displaying the data it is reading from the pre-recorded data over the topic "chatter". Once the playback finishes, you may ternimate all instances of ros.

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
