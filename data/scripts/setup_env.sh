#!/bin/bash

cd /home/sora/autoware

sudo apt update

rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

sudo ip link set lo multicast on
