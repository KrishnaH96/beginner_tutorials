# /**
#  * @file nodes_launch.py
#  * @brief Launch file to launch nodes all together.
#  *
#  * This file contains a Python launch script to launch the publisher (talker) node from the terminal.
#  *
#  * @author Krishna Rajesh Hundekari
#  * @date 2023-11-
#  * @copyright 2023 Krishna Rajesh Hundekari
#  * Apache License Version 2.0, January 2004
#  * Licensed to the Apache Software Foundation (ASF) under one
#  * or more contributor license agreements.  See the NOTICE file
#  * distributed with this work for additional information
#  * regarding copyright ownership.  The ASF licenses this file
#  * to you under the Apache License, Version 2.0 (the
#  * "License"); you may not use this file except in compliance
#  * with the License.  You may obtain a copy of the License at
#  *
#  *   http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing,
#  * software distributed under the License is distributed on an
#  * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  * KIND, either express or implied.  See the License for the
#  * specific language governing permissions and limitations
#  * under the License.
#  */

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument for publish frequency
    publisher_freq_launch = DeclareLaunchArgument(
        "publish_frequency", default_value='1000'
    )

    return LaunchDescription([
        publisher_freq_launch,
        # Launch the talker node with the specified publish frequency
        Node(
            package='first_ros_package', 
            executable='talker',
            parameters=[{"pub_freq": LaunchConfiguration('publish_frequency')}]  # Pass the publish frequency in milliseconds
        )  
    ])
