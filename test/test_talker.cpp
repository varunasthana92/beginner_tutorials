/**
 * @file test_talker.cpp
 * @brief unit test file for talker node
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

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>

/*
 * Declaring test fixture CheckTest that will subscribe to topic
 * "chatter" published by node under testing
 */
class CheckTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh;
  std::string a;
  void SetUp() override {
    auto sub = nh.subscribe("chatter", 100, &CheckTest::testChatterCallback,
                            this);
    int i = 1;
    ros::Rate rate(5);
    /*
     * while loop used to have a delay inorder for publisher and subscriber
     * to connect
     */
    while (i < 10) {
      ros::spinOnce();
      ++i;
      rate.sleep();
    }
  }
  /*
   * Callback function for the subscriber
   * Simply to listen to the topic and assign published data to
   * a object of type std::string
   */
  void testChatterCallback(const std_msgs::String::ConstPtr &msg);
};

void CheckTest::testChatterCallback(const std_msgs::String::ConstPtr &msg) {
  a = msg->data;
  return;
}

TEST_F(CheckTest, testTalker) {
  /*
   * testing the size of string published. It is known string
   * toggles between "Varun" and "Asthana". hence string size
   * comparison is done for range between 4 and 8 as string length
   * of "varun" is 5 and of "Asthana" is 7
   */
  EXPECT_GT(a.size(), 4);
  EXPECT_LT(a.size(), 8);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talker_test");
  return RUN_ALL_TESTS();
}
