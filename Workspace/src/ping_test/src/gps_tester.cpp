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



int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "gps_tester");
    ros::NodeHandle nh;

    //Set up publishers and subscribers
    ros::Publisher gpsPub=nh.advertise<sensor_msgs::NavSatFix>(GPS_SENSOR_TOPIC, 1);

    sensor_msgs::NavSatFix msg;

    msg.header.stamp = ros::Time::now();

    msg.longitude = 0;
    msg.latitude = 0;
    msg.altitude = 0;

    ros::Rate rate(1);
    while (ros::ok())
    {
        gpsPub.publish(msg);

        msg.longitude += 0.1;
        rate.sleep();
    }

}
