#!/bin/bash

cd ~/ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon test
colcon test-result --verbose