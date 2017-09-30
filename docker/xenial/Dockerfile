FROM ubuntu:xenial

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

ENV CONTINUOUS_INTEGRATION true
ENV ROS_DISTRO kinetic

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    make \
    gcc \
    g++ \
    locales \
    wget \
    software-properties-common \
    sudo

ADD /scripts /scripts

RUN bash /scripts/ros_install.bash

RUN bash /scripts/create_catkin_workspace.bash
