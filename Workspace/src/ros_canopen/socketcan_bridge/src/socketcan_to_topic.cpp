/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/sensor_data.h>
#include <socketcan_interface/string.h>
#include <can_msgs/Frame.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <linux/can.h>


namespace socketcan_bridge
{
    SocketCANToTopic::SocketCANToTopic(boost::shared_ptr<can::DriverInterface> driver)
    {
        driver_ = driver;
    }

    SocketCANToTopic::~SocketCANToTopic()
    {
        this->cleanup();
    }

    void SocketCANToTopic::cleanup()
    {
        nh_.shutdown();

        // Deallocate memory for publisher pointers
        for (auto& topic_pair : topics_)
        {
            topic_pair.second.reset();
        }

        topics_.clear();

        ROS_INFO("CAN receiver publishers deallocated and vectors cleared");
    }

    void SocketCANToTopic::init()
    {
        // register handler for frames and state changes.
        frame_listener_ = driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback));
        state_listener_ = driver_->createStateListener(can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));

        // Get parameters from .yaml file
        this->getParams(nh_);

        if (!this->publishTopics(topics_))
        {
            ROS_WARN("Could not publish topics");
            return;
        }
        ROS_INFO("CAN receiver initialization complete");
    }

    void SocketCANToTopic::getParams(ros::NodeHandle& nh)
    {
        nh.getParam("/incoming_ids", message_ids_);
        std::vector<std::string> names;
        nh.getParam("/output_topics", names);
        for (std::string name : names)
        {
            topics_.insert(std::pair<std::string, std::unique_ptr<ros::Publisher>>(name, std::unique_ptr<ros::Publisher>(nullptr)));
        }
    }

    bool SocketCANToTopic::publishTopics(std::map<std::string, std::unique_ptr<ros::Publisher>>& topic_list)
    {
        if (topic_list.size() == 0)
        {
            return false;
        }

        for (auto& topic_pair : topic_list)
        {
            std::string topic_name = topic_pair.first;

            // Change publisher type based on topic
            if (topic_name == "/absoluteEncoders")
            {
                topic_pair.second.reset();
                topic_pair.second = std::unique_ptr<ros::Publisher>(new ros::Publisher);
                *topic_pair.second = nh_.advertise<std_msgs::UInt16MultiArray>(topic_name, 10);
                encoder_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
                encoder_msg_.layout.dim[0].size = 5;
                encoder_msg_.layout.dim[0].stride = 1;
                encoder_msg_.layout.dim[0].label = "absoluteEncoders";
                encoder_msg_.data.resize(5);
            }
            else if (topic_name == "/limitSwitches")
            {
                topic_pair.second.reset();
                topic_pair.second = std::unique_ptr<ros::Publisher>(new ros::Publisher);
                *topic_pair.second = nh_.advertise<std_msgs::UInt32>(topic_name, 10);
            }
            else
            {
                topic_pair.second.reset();
                ROS_WARN("Topic name \"%s\" is not registered in config file", topic_name.c_str());
                return false;
            }
        }

        return true;
    }

    void SocketCANToTopic::frameToMessage(const can::Frame& f, can_msgs::Frame& m)
    {
        m.id = f.id;
        m.dlc = f.dlc;
        m.is_error = f.is_error;
        m.is_rtr = f.is_rtr;
        m.is_extended = f.is_extended;

        for (int i = 0; i < m.dlc; i++)  // copy data based on dlc
        {
            m.data[i] = f.data[i];
        }
    }

    void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
        can::Frame frame = f;  // copy the frame first, cannot call isValid() on const.
        if (!frame.isValid())
        {
            ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d",
                      f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
            return;
        }
        else
        {
            if (f.is_error)
            {
                // can::tostring cannot be used for dlc > 8 frames. It causes an crash
                // due to usage of boost::array for the data array. The should always work.
                ROS_WARN("Received error frame: %s", can::tostring(f, true).c_str());
                return;
            }
            if (f.is_rtr)
            {
                ROS_WARN("Received RTR frame: RTR frames not supported %s", can::tostring(f, true).c_str());
            }
            if (f.is_extended)
            {
                ROS_WARN("Received extended frame: extended frames not supported %s", can::tostring(f, true).c_str());
            }
        }

        can_msgs::Frame msg;
        // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
        this->frameToMessage(frame, msg);

        msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
        msg.header.stamp = ros::Time::now();

        // check if message ID exists inside map
        if (message_ids_.count(std::to_string(msg.id)))
        {
            std::string message_type = message_ids_[std::to_string(msg.id)];
            if (message_type == "limitSwitches")
            {
                // will always be uint32_t
                if (msg.dlc != 4)
                {
                    ROS_WARN("Incorrect data length for limit switch message");
                    return;
                }
                uint32_t limit_switch = 0;
                limit_switch = msg.data[3] << 24 | msg.data[2] << 16 | msg.data[1] << 8 | msg.data[0];
                limit_switch_msg_.data = limit_switch;
                topics_["/limitSwitches"]->publish(limit_switch_msg_);
            }
            else if (message_type == "armJointEncoder1" ||
                     message_type == "armJointEncoder2" ||
                     message_type == "armJointEncoder3" ||
                     message_type == "armJointEncoder4" ||
                     message_type == "armJointEncoder5")
            {
                // will always be uint16_t
                if (msg.dlc != 2)
                {
                    ROS_WARN("Incorrect data length for arm encoder message");
                    return;
                }
                uint16_t encoder_value;
                encoder_value = msg.data[1] << 8 | msg.data[0];
                encoder_mutex_.lock();
                encoder_msg_.data[msg.id % 300] = encoder_value; // all encoder message IDs are in the 300 range
                encoder_mutex_.unlock();
                topics_["/absoluteEncoders"]->publish(encoder_msg_);
            }
            else if (message_type == "endEffectorEncoder")
            {
            }
            else
            {
                ROS_WARN("Message ID is not recognized");
            }
        }
    }

    void SocketCANToTopic::stateCallback(const can::State & s)
    {
        std::string err;
        driver_->translateError(s.internal_error, err);
        if (!s.internal_error)
        {
            ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
        }
        else
        {
            ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
        }
    }
};  // namespace socketcan_bridge
