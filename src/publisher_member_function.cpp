/**
 * @file publisher_member_function.cpp
 * @brief A minimal example of a ROS 2 publisher using rclcpp.
 *
 * This file contains the implementation of a minimal ROS 2 publisher node that
 * publishes custom string messages to a topic at a fixed rate.
 *
 * @author Krishna Rajesh Hundekari
 * @date 2023-11-21
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "first_ros_package/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


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
    // Set the default message data
    message.data = "No Custom Name yet... Default John Doe....";

    // Log a debug message with the initial message data
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Custom message that will be published: " + message.data);

    // Declare and get the publishing frequency parameter
    this->declare_parameter<int>("pub_freq", 1000);
    int pub_freq_ = this->get_parameter("pub_freq").as_int();

    // Log the current publishing frequency
    RCLCPP_INFO_STREAM(this->get_logger(), "Current frequency: " << pub_freq_);

    // Check and log warnings, fatal errors, or errors based on the publishing
    // frequency
    if (pub_freq_ == 500) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Same as default frequency that is " << pub_freq_);
    }

    if (pub_freq_ >= 1000) {
      RCLCPP_FATAL_STREAM(this->get_logger(),
                          "It's too slow, fatal error.... ");
    }

    if (pub_freq_ < 50) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "It's too fast, data loss warning.... ");
    }

    // Create publisher, timer, service and tf_broadcaster.
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_freq_),
        std::bind(&MinimalPublisher::timer_callback, this));

    service_ = this->create_service<first_ros_package::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::changeString, this, std::placeholders::_1,
                  std::placeholders::_2));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  /**
   * @brief Timer callback function for publishing messages at a fixed rate.
   */
  void timer_callback() {
    // Log an info message with the data being published
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    publish_transform_();
  }

  /**
   * @brief Callback function for handling service requests to change the
   * string.
   * @param request The service request.
   * @param response The service response.
   */
  void changeString(
      const std::shared_ptr<first_ros_package::srv::ChangeString::Request>
          request,
      std::shared_ptr<first_ros_package::srv::ChangeString::Response>
          response) {
    // Concatenate first and last names from the service request
    response->full_name = request->first_name + " " + request->last_name;

    // Log an info message with the processed service request and response
    RCLCPP_INFO(this->get_logger(), "Service request processed. Response: '%s'",
                response->full_name.c_str());

    // Update the message data with the response for future publishing
    message.data = response->full_name;
  }

/**
 * @brief Publish a transform from the "world" frame to the "talk" frame.
 *
 * This function publishes a transform using the TF2 broadcaster from the
 * "world" frame to the "talk" frame with a translation of (5.0, 5.0, 0.0)
 * and a rotation of (1.57, 1.57, 0).
 */
void publish_transform_() {
  // Create a TransformStamped message
  geometry_msgs::msg::TransformStamped t;

  // Set the header information
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";

  // Set the translation
  t.transform.translation.x = 5.0;
  t.transform.translation.y = 5.0;
  t.transform.translation.z = 0.0;

  // Set the rotation using Roll-Pitch-Yaw angles
  tf2::Quaternion q;
  q.setRPY(1.57, 1.57, 0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transform using the TF2 broadcaster
  tf_broadcaster_->sendTransform(t);
}

  // Shared pointer to the timer for publishing messages at a fixed rate.
  rclcpp::TimerBase::SharedPtr timer_;

  // Shared pointer to the publisher for publishing string messages.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Shared pointer to the service for handling string changes.
  rclcpp::Service<first_ros_package::srv::ChangeString>::SharedPtr service_;

  // Variable to keep track of a count or index, not currently used.
  size_t count_;

  // String message variable to store and publish custom string messages.
  std_msgs::msg::String message;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

/**
 * @brief The main function that initializes the ROS 2 node and runs the
 * publisher.
 * @param argc The number of command line arguments.
 * @param argv An array of command line arguments.
 * @return An integer status code.
 */
int main(int argc, char *argv[]) {
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  // Create an instance of the MinimalPublisher class
  auto minimal_publisher = std::make_shared<MinimalPublisher>();

  // Spin the node, i.e., keep it active and responsive to events
  rclcpp::spin(minimal_publisher);

  // Shutdown the ROS 2 node
  rclcpp::shutdown();

  // Return an integer status code
  return 0;
}
