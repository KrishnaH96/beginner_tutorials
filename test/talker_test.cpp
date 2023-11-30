/**
 * @file talker_test.cpp
 * @brief gtest script to test the first_ros_package talker node.
 *
 * @author Krishna Rajesh Hundekari
 * @date 2023-11-27
 * @copyright 2023 Krishna Rajesh Hundekari
 * Apache License Version 2.0, January 2004
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership. The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Test fixture for testing the talker node in the first_ros_package.
 */
class CustomTestFixture : public testing::Test {
 public:
  /**
   * @brief Constructor for the test fixture.
   */
  CustomTestFixture()
      : test_node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(test_node_->get_logger(), "INITIALIZED!!");
  }

  /**
   * @brief Set up function called before each test case.
   */
  void SetUp() override {
    // cppcheck-suppress unusedFunction
    bool start_result =
        StartROSNode("first_ros_package", "minimal_publisher", "talker");
    ASSERT_TRUE(start_result);

    RCLCPP_INFO_STREAM(test_node_->get_logger(), "SETUP COMPLETE!!");
  }

  /**
   * @brief Tear down function called after each test case.
   */
  void TearDown() override {
    // cppcheck-suppress unusedFunction
    bool stop_result = StopROSNode();
    ASSERT_TRUE(stop_result);

    std::cout << "TEARDOWN COMPLETE" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr test_node_;
  std::stringstream command_stream, node_info_stream, kill_command_stream;

  /**
   * @brief Start the ROS node with the specified package, node, and executable
   * names.
   *
   * @param pkg_name The ROS package name.
   * @param node_name The ROS node name.
   * @param exec_name The ROS executable name.
   * @return True if the ROS node is started successfully, false otherwise.
   */
  bool StartROSNode(const char *pkg_name, const char *node_name,
                    const char *exec_name) {
    command_stream << "ros2 run " << pkg_name << " " << exec_name
                   << " > /dev/null 2> /dev/null &";
    node_info_stream << "ros2 node info "
                     << "/" << node_name << " > /dev/null 2> /dev/null";
    char exec_name_buffer[16];
    snprintf(exec_name_buffer, sizeof(exec_name_buffer), "%s", exec_name);
    kill_command_stream << "pkill --signal SIGINT " << exec_name_buffer
                        << " > /dev/null 2> /dev/null";

    StopROSNode();

    int result = system(command_stream.str().c_str());
    if (result != 0) return false;

    result = -1;
    while (result != 0) {
      result = system(node_info_stream.str().c_str());
      sleep(1);
    }
    return true;
  }

  /**
   * @brief Stop the ROS node.
   *
   * @return True if the ROS node is stopped successfully, false otherwise.
   */
  bool StopROSNode() {
    if (kill_command_stream.str().empty()) return true;

    int result = system(kill_command_stream.str().c_str());
    return result == 0;
  }
};

/**
 * @brief Test case to ensure that the TrueIsTrueTest is functioning as
 * expected.
 */
TEST_F(CustomTestFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  using std_msgs::msg::String;
  using SubscriberType = rclcpp::Subscription<String>::SharedPtr;
  bool has_data = false;
  SubscriberType subscription = test_node_->create_subscription<String>(
      "topic", 10,

      [&](const std_msgs::msg::String &msg) {
        RCLCPP_INFO(test_node_->get_logger(), "I heard: '%s'",
                    msg.data.c_str());
        has_data = true;
      });

  using TimerType = std::chrono::system_clock;
  using namespace std::chrono_literals;
  TimerType::time_point start_time;
  TimerType::duration elapsed_time;
  start_time = TimerType::now();
  elapsed_time = TimerType::now() - start_time;
  rclcpp::Rate loop_rate(2.0);
  while (elapsed_time < 3s) {
    rclcpp::spin_some(test_node_);
    loop_rate.sleep();
    elapsed_time = TimerType::now() - start_time;
  }
  EXPECT_TRUE(has_data);
}

/**
 * @brief Main function to initialize ROS, run tests, and shut down ROS.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Test result code.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int test_result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "ROS SHUTDOWN COMPLETE" << std::endl;
  return test_result;
}
