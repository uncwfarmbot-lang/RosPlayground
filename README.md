# RosPlayground
This is our early set up for the implementation of Ros2 on a raspberry pi. This repo contains the nodes and early setup of how we constructed this.


# FarmBot Stepper Motor Control with ROS2

ROS2 package for controlling stepper motors on Raspberry Pi 4 (Ubuntu 22.04).  
Includes multiple nodes that demonstrate **ROS2 publishers, subscribers, and parameters**.

## Features
- Continuous rotation (`stepper_forever.py`)
- Parameterized steps/direction (`stepper_variable.py`)
- Subscriber node listening on `/motor_command` (`stepper_motor.py`)
- Keyboard-based publisher (`keyboard_controller.py`)

## Prerequisites
- Raspberry Pi 4 (tested on Ubuntu 22.04)
- ROS2 Humble
- Python 3.10+
- Stepper driver (e.g., A4988, DM542)

## Quick Start
```bash
# Clone repo
git clone https://github.com/YOUR-USERNAME/farmbot-stepper-ros2.git
cd farmbot-stepper-ros2/ros2_ws

# Build
colcon build
source install/setup.bash

# Run continuous rotation
ros2 run stepper_pkg stepper_forever
