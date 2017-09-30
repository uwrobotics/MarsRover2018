#!/bin/bash
set -e  # exit on first error

cd "$HOME/catkin_ws"

if [ -z "$CONTINUOUS_INTEGRATION" ]; then
    catkin build --make-args tests
    catkin build --make-args test
else
    catkin build --no-status --make-args tests
    catkin build --no-status --make-args test
fi
