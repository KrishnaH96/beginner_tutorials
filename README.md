# beginner_tutorials

## Overview
This repository contains a ros2 package(C++) with two nodes, one publishier and one subscribing node.
Publisher publishes a message on the topic and subscriber by subscribing to that topic recieves that messages and prints on the terminal.

## Dependencies
* ROS 2 Humble
* Ubuntu 22.04

## Build Instructions
```
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src

git clone https://github.com/KrishnaH96/beginner_tutorials.git

cd ..

rosdep install -i --from-path src --rosdistro humble -y

colcon build 

```

## Run instructions

### First terminal:

```
source /opt/ros/humble/setup.bash

cd ros2_ws

source install/local_setup.bash

ros2 run cpp_pubsub listener

```

### In Second Terminal:

```
source /opt/ros/humble/setup.bash

cd ros2_ws

source install/local_setup.bash

ros2 run cpp_pubsub talker

```
