#!/bin/bash

dir=$(pwd)

docker run -t -v ${dir}:/MarsRover2018 uwrobotics/ubuntu:${1} /bin/bash /MarsRover2018/docker/run-ci-tests.sh

if [ $? -ne 0 ]; then
    echo "Error: ${1} docker run failed"
    exit 1
fi
