/**
 * @file publisher_member_function.cpp
 * @brief A minimal example of a ROS 2 publisher using rclcpp.
 *
 * This file contains the implementation of a minimal ROS 2 publisher node that
 * publishes custom string messages to a topic at a fixed rate.
 *
 * @author Krishna Rajesh Hundekari
 * @date 2023-11-07
 * @copyright 2023 Krishna Rajesh Hundekari
* Apache License Version 2.0, January 2004

  * Licensed to the Apache Software Foundation (ASF) under one
  * or more contributor license agreements.  See the NOTICE file
  * distributed with this work for additional information
  * regarding copyright ownership.  The ASF licenses this file
  * to you under the Apache License, Version 2.0 (the
  * "License"); you may not use this file except in compliance
  * with the License.  You may obtain a copy of the License at
  *
  *   http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing,
  * software distributed under the License is distributed on an
  * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
  * KIND, either express or implied.  See the License for the
  * specific language governing permissions and limitations
  * under the License.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class MinimalPublisher
 * @brief A minimal ROS 2 publisher node.
 *
 * This class represents a ROS 2 publisher node that publishes custom string
 * messages to a specified topic at a regular interval.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the MinimalPublisher class.
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback function for publishing messages at a fixed rate.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data =
        "This is a custom string Message!!" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

/**
 * @brief The main function that initializes the ROS 2 node and runs the
 * publisher.
 * @param argc The number of command line arguments.
 * @param argv An array of command line arguments.
 * @return An integer status code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
