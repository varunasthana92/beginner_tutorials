/**
 * @file talker.cpp
 * @brief publisher for First Publisher/Subscriber ros package
 * @author Varun Asthana
 *
 * Copyright [2019] Varun Asthana
 * All rights reserved.
 *
 * BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ChangeString.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int freq;
  const std::string paramName = "~freq";
  bool check = ros::param::get(paramName, freq);
  /**
   * Conditional statement to check if the required parameter
   * for publish frequency on the topic "chatter" has been
   * defined or not. If defined, a verification put to see that the
   * rate is not set to zero.
   */
  if (!check) {
    while (ros::ok()) {
      ROS_FATAL_STREAM("Could not get parameter " << paramName);
    }
    return 0;
  } else if (freq <= 0) {
    while (ros::ok()) {
      ROS_WARN_STREAM("Publish rate should be more than zero!!!");
    }
    return 0;
  }
  ros::Rate loop_rate(freq);
  ROS_DEBUG_STREAM("Publish rate set to " << freq);
  /**
   * A service client setup to send request to a
   * service named "change_string_output"
   */
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::ChangeString>(
      "change_string_output");
  beginner_tutorials::ChangeString srv;
  /**
   * A count of how many messages we have sent.
   */
  int count = 0;
  bool a = true;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    srv.request.choice = a;
    /**
     * Call to service made and verifying if successful or not.
     * If not, then a ROS_ERROR generated.
     */
    if (client.call(srv)) {
      msg.data = srv.response.name;
      ROS_INFO_STREAM(msg << " Run count: " << count);
      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      chatter_pub.publish(msg);
    } else {
      ROS_ERROR_STREAM("Failed to call service change_string_output");
    }
    /**
     * Toggling the bool value of "a" to change output string in next call
     * to the service.
     */
    a = !a;
    transform.setOrigin(tf::Vector3(1.0, 2.0, 0.0));
    /**
     * Setting rotation as 180 degrees
     * i.e. 3.1415 radians (approx)
     */
    tf::Quaternion q;
    q.setRPY(0, 0, 3.1415);
    transform.setRotation(q);
    /*
     * Broadcasting the "talk" tf with "world" frame as its parent
     */
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 1;
}

