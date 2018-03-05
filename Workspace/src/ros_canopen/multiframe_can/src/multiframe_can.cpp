/*********************************************************
    __  ___                   ____
   /  |/  /___ ___________   / __ \____ _   _____  _____
  / /|_/ / __ `/ ___/ ___/  / /_/ / __ \ | / / _ \/ ___/
 / /  / / /_/ / /  (__  )  / _, _/ /_/ / |/ /  __/ /
/_/  /_/\__,_/_/  /____/  /_/ |_|\____./|___/\___/_/

Copyright 2017, UW Robotics Team

@file     multiframe_can.cpp
@author:  Archie Lee

**********************************************************/

#include <multiframe_can/multiframe_can.h>

/* Multiframe_CAN()
/
/   Description:
/       Class constructor
/
/   Arguments:
/       None
/
/   Returns:
/       None
*/
void Multiframe_CAN::Multiframe_CAN() {
  multiframe_can::data_ready_ = false;
  multiframe_can::flow_status_ = 0;
  multiframe_can::block_size_ = 0;
  multiframe_can::separation_time_ = 0;
  multiframe_can::frame_data_.clear();
}

/* publish()
/
/   Description:
/       Wraps a given byte vector and node ID in a ROS message and publishes it
with a given ROS publisher
/       Must provide a valid publisher that publishes to a topic recognized by
socketcan_bridge
/
/   Arguments:
/       ros::Publisher *pub         ROS publisher
/       std::vector<char>& msg      Byte vector
/       int id                      Node ID
/
/   Returns:
/       0 on success
/       1 if given msg is too long
/       2 if given publisher does not exist
*/
char Multiframe_CAN::publish(ros::Publisher *pub, std::vector<char> &msg,
                             int id) {
  int num_bytes = msg.size();
  int msg_index = 0;
  can::Frame frame;

  if (num_bytes > MAX_MSG_SIZE) {
    // error: msg too long
    return 1;
  }
  if (!pub) {
    // publisher doesn't exist
    return 2;
  }

  // static, these parameters won't change between frames
  frame.id = id;
  frame.is_rtr = false;
  frame.is_error = false;
  frame.is_extended = false;

  // single frame msg
  if (num_bytes <= 7) {
    frame.data[0] = (num_bytes & 0xF);
    for (int i = 1; i < 8; i++) {
      frame.data[i] = msg[i - 1];
    }

    pub->publish(frame);
  }
  // multiframe msg
  else {
    int start;
    while (msg_index < num_bytes) {
      // first frame
      if (!msg_index) {
        frame.data[0] = (1 << 4) | ((num_bytes >> 8) & 0xF);
        frame.data[1] = (num_bytes & 0xFF);
        start = 2;
      }
      // consecutive frames
      else {
        frame.data[0] = (2 << 4) | (frame_number & 0xF);
        start = 1;
      }

      // calculate data length code
      // should be 8 unless it is the last frame, then it should be 1 +
      // whatever's leftover
      frame.dlc = (msg_index + 7 < num_bytes) ? 8 : (num_bytes - msg_index) + 1;

      // populate data
      for (int i = start; i < frame.dlc; i++) {
        frame.data[i] = msg[msg_index];
      }

      // move msg_index forward
      if (!msg_index) {
        msg_index += 6;
      } else {
        msg_index += 7;
      }

      // publish frame
      pub->publish(frame);
    }
  }

  return 0;
}

/* subscribe()
/
/   Description:
/       Parses a given CAN frame ROS message
/       If successful, stores the data in class' byte vector named "frame_data_"
/       On a malformed frame error, the class' byte vector will be cleared
/
/   Arguments:
/       can::Frame msg          Frame to be parsed
/
/   Returns:
/       0 when the vector is ready to be accessed
/       1 when the vector is not fully populated
/       2 if vector is fully populated and not yet read
/       3 if the frame is not formatted corrected (WILL ALSO CLEAR THE BYTE
VECTOR)
*/
char Multiframe_CAN::subscribe(can::Frame msg) {
  int frame_type = msg.data[0] & 0xF0;
  int start;
  bool malformed = false;
  static int num_bytes = 0;
  static int curr_bytes = 0;
  static int curr_frame = 0;

  // only proceed if data is not ready (if ready, don't want to overwrite byte
  // vector)
  if (!multiframe_can::data_ready_) {
    switch (frame_type) {
    case SINGLE_FRAME:
      // ensure the data length code is correct and variables are not populated
      if (msg.dlc == 8 && !curr_bytes && !num_bytes) {
        start = 1;
        multiframe_can::data_ready_ = true;
      }
      // data length code is incorrect or variables already populated
      else {
        malformed = true;
      }
      break;

    case FIRST_FRAME:
      // ensure the data length code is correct and variables are not populated
      if (msg.dlc == 8 && !curr_bytes && !num_bytes) {
        start = 2;
        num_bytes = (msg.data[0] & 0xF) << 8 | msg.data[1];
        curr_bytes = 6;
        curr_frame = 0;
      }
      // data length code is incorrect or variables already populated
      else {
        malformed = true;
      }
      break;

    case CONSECUTIVE_FRAME:
      // consecutive frame means these variables must be populated in case 1
      if (!curr_bytes || !num_bytes) {
        malformed = true;
      } else {
        start = 1;
        curr_frame++;
        // must ensure that the frames are in order
        if (curr_frame == (msg.data[0] & 0xF)) {
          // not final frame
          if ((curr_bytes + 7 < num_bytes) && (msg.dlc == 8)) {
            curr_bytes += 7;
          }
          // final frame
          else if ((curr_bytes + 7 >= num_bytes) &&
                   (num_bytes - curr_bytes == msg.dlc - 1)) {
            multiframe_can::data_ready_ = true;
          }
          // msg's data length code is incorrect
          else {
            malformed = true;
          }
        }
        // out of sequence frame received
        else {
          malformed = true;
        }
      }
      break;

    // flow control frame --> not implemented yet
    case FLOW_CONTROL:
    // erroneous frame
    default:
      malformed = true;
      break;
    }

    // on malformed frame, reset the byte vector and static variables
    if (malformed) {
      curr_bytes = 0;
      num_bytes = 0;
      curr_frame = 0;
      multiframe_can::frame_data_.clear();
      return 3;
    }

    // populate byte vector
    for (int i = start; i < msg.dlc; i++) {
      multiframe_can::frame_data_.push_back(msg.data[i]);
    }

    // byte vector fully populated
    if (multiframe_can::data_ready_) {
      // reset static counters before next function call
      curr_bytes = 0;
      num_bytes = 0;
      curr_frame = 0;
      return 0;
    }

    // successful but byte vector not ready
    return 1;
  }
  return 2;
}

/* get_data()
/
/   Description:
/       Accessor for the frame_data byte vector which stores the combined data
/       Should only be called when subscribe() returns 0 or the byte vector will
be empty
/       If successful, clears the byte vector to prepare for the next multiframe
message
/
/   Arguments:
/       None
/
/   Returns:
/       Class' byte vector containing combined data if data is ready
*/
std::vector<char> Multiframe_CAN::get_data() {
  std::vector<char> data = std::vector<char>();
  // only return frame_data_ vector if data is ready
  if (multiframe_can::data_ready_) {
    multiframe_can::data_ready_ = false;
    data = multiframe_can::frame_data_;
    multiframe_can::frame_data_.clear();
  }
  return data;
}
