/*********************************************************
    __  ___                   ____
   /  |/  /___ ___________   / __ \____ _   _____  _____
  / /|_/ / __ `/ ___/ ___/  / /_/ / __ \ | / / _ \/ ___/
 / /  / / /_/ / /  (__  )  / _, _/ /_/ / |/ /  __/ /
/_/  /_/\__,_/_/  /____/  /_/ |_|\____./|___/\___/_/

Copyright 2017, UW Robotics Team

@file     multiframe_can.h
@author:  Archie Lee

**********************************************************/

/*
  Every CAN frame has 8 bytes of data. This framework allows users to transmit
  data larger than 8 bytes over CAN.

  For multiframe publishing, ensure that your message is in a byte vector.
*/

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <vector>

#define MAX_MSG_SIZE 64

#define SINGLE_FRAME 0
#define FIRST_FRAME 1
#define CONSECUTIVE_FRAME 2
#define FLOW_CONTROL 3

namespace multiframe_can {
class Multiframe_CAN {
public:
  Multiframe_CAN();
  char publish(ros::Publisher *pub, std::vector<char> &msg, int id);
  char subscribe(can::Frame msg);
  std::vector<char> get_data();

private:
  bool data_ready_;
  int flow_status_;
  int block_size_;
  int separation_time_;
  std::vector<char> frame_data_;
};
};
