#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <iostream>
#include "timer.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "ping_test.h"
#include <ctime>

enum ePingState {
    UNSTARTED,
    CONNECTED,
    DISCONNECTED
};

ePingState currentPingState = ePingState::UNSTARTED;

//GPS location of the last location with known connectivity
sensor_msgs::NavSatFix::ConstPtr lastGoodGPS = nullptr;
//Last GPS message received
sensor_msgs::NavSatFix::ConstPtr mostRecentGPS = nullptr;

//mutex to protect the above info
std::mutex pingGPSMutex;

void GPSMessageCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg)
{
    std::lock_guard<std::mutex> lock(pingGPSMutex);
    mostRecentGPS = gpsMsg;
}


void PingResultCallback(const std_msgs::Bool::ConstPtr& pingMsg)
{
    if (pingMsg->data == true)
    {
        std::lock_guard<std::mutex> lk(pingGPSMutex);
        lastGoodGPS = mostRecentGPS;
        currentPingState = ePingState::CONNECTED;
    }
    else
    {
        std::lock_guard<std::mutex> lk(pingGPSMutex);
        currentPingState = ePingState::DISCONNECTED;
    }
}


int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "ping_GPS_tracker");
    ros::NodeHandle nh;

    //Set up publishers and subscribers
    ros::Publisher gpsPub=nh.advertise<sensor_msgs::NavSatFix>(GPS_TARGET_TOPIC, 1);
    ros::Subscriber PingResultSub=nh.subscribe(PING_CLIENT_RESULT_TOPIC, 1, PingResultCallback);
    ros::Subscriber gpsResultSub=nh.subscribe(GPS_SENSOR_TOPIC, 1, GPSMessageCallback);

    //loop once per second: makes sure that GPS message doesn't get overwiten
    //when back-tracking
    //and limits cpu use when connected
    ros::Rate loopRate(1);

    while (ros::ok())
    {
        //Spin twice in case both subscribers received something
        ros::spinOnce();

        std::unique_lock<std::mutex> lk(pingGPSMutex);
        switch (currentPingState)
        {
        case ePingState::UNSTARTED:
            break;//Do nothing until we have some info
        case ePingState::CONNECTED:
            break;//Do nothing, we are fine
        case ePingState::DISCONNECTED:
            if (lastGoodGPS != nullptr)
            {
                gpsPub.publish(*lastGoodGPS);
            }
            break;
        default:
            break;
        }

        lk.unlock();
        loopRate.sleep();
    }

    return 0;
}
