#!/bin/bash
set -e
export UBUNTU_CODENAME=$(lsb_release -s -c)
case $UBUNTU_CODENAME in
  xenial)
    export ROS_DISTRO=kinetic;;
  *)
    echo "Unsupported version of Ubuntu detected. Only xenial (16.04.*) is currently supported."
    exit 1
esac
export REPO_DIR=$(dirname "$SCRIPT_DIR")
export CATKIN_DIR="$HOME/catkin_ws"
