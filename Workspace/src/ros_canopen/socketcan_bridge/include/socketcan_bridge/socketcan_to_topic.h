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

#ifndef SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H
#define SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H

#include <socketcan_interface/socketcan.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <can_node_ids/can_rx_id.h>
#include <vector>
#include <string>

namespace socketcan_bridge
{
    class SocketCANToTopic
    {
        public:
            SocketCANToTopic(boost::shared_ptr<can::DriverInterface> driver);
            ~SocketCANToTopic();
            void init();
            void cleanup();

        private:
            ros::NodeHandle nh_;
            boost::shared_ptr<can::DriverInterface> driver_;
            std::vector<ros::Publisher*> topics_;
            std::vector<std::string> topic_names_;

            can::CommInterface::FrameListener::Ptr frame_listener_;
            can::StateInterface::StateListener::Ptr state_listener_;

            void getParams(ros::NodeHandle& nh);
            void frameToMessage(const can::Frame& f, can_msgs::Frame& m);
            void frameCallback(const can::Frame& f);
            void stateCallback(const can::State& s);
            bool publishTopic(std::vector<std::string>& topic_list);
    };
};  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE_SOCKETCAN_TO_TOPIC_H
