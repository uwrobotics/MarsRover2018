#!/bin/bash

sudo apt-get install ros-kinetic-hector-gazebo
cd /opt/ros/kinetic/share
git clone https://github.com/husky/husky.git
cd husky
git checkout kinetic-devel
