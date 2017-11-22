#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <iostream>
#include "timer.h"
#include "ping_test.h"
#include <thread>
#include <mutex>
#include <chrono>
Timer t;

ros::Publisher replyPub;
ros::Publisher resultPub;

static int pingCount = 0;
std::mutex pingCountMutex;

bool resetting = false;

unsigned int timestamp() {
	using namespace std::chrono;
	return duration_cast< milliseconds >(
			    system_clock::now().time_since_epoch()).count();
}

void timerExpired(const boost::system::error_code& ec) {
	if (ec == boost::asio::error::operation_aborted) {
		//No-op
	} else {
		std::cout << "Timer expired\n";

        std_msgs::Bool panicMsg;
        panicMsg.data = false;
        resultPub.publish(panicMsg);
        t.restartTimer();
	}
}

void requestCallback(const std_msgs::UInt8MultiArray::ConstPtr& requestMsg)
{
	if (resetting) {
		return;
	}
	t.cancel();
	std::cout << "Received ping\n";
	std_msgs::UInt8MultiArray replyMsg;
    std_msgs::Bool resultMsg;
	
	if (requestMsg->data.size() == PING_BYTES_PER_TEST) {
		replyMsg.data.reserve(PING_BYTES_PER_TEST);
		for (int i = 0; i < PING_BYTES_PER_TEST; i++) {
			replyMsg.data.push_back(requestMsg->data[i]);

		}
		replyPub.publish(replyMsg);

        resultMsg.data = true;
		t.start(2*PING_TIME_BETWEEN_TESTS, &timerExpired);

	} else {
        resultMsg.data = false;
    }
    resultPub.publish(resultMsg);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "ping_test_server");
	ros::NodeHandle nh;
	replyPub=nh.advertise<std_msgs::UInt8MultiArray>(PING_REPLY_TOPIC, 1);
	ros::Subscriber sub=nh.subscribe(PING_REQUEST_TOPIC, 1,  requestCallback);
	resultPub=nh.advertise<std_msgs::Bool>(PING_SERVER_RESULT_TOPIC, 1);

	t.start(2*PING_TIME_BETWEEN_TESTS, &timerExpired);
	ros::spin();


	return 0;
}
