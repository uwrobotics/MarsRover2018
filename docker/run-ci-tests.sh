#!/bin/bash
set -e  # exit on first error

source /opt/ros/${ROS_DISTRO}/setup.bash

bash /MarsRover2018/scripts/build_and_run_unit_tests.bash
