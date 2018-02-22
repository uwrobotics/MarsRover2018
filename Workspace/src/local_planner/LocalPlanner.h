//
// Created by tom on 20/02/18.
//

#ifndef PROJECT_LOCALPLANNER_H
#define PROJECT_LOCALPLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>

class CLocalPlanner
{
public:
    explicit CLocalPlanner(ros::NodeHandle* pNh);

private:
    //Subscriber callbacks
    void CurVelCallback(geometry_msgs::Twist::ConstPtr vel);
    void GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal);
    void CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps);
    void OccupancyCallback();


    //Ros handlers
    ros::NodeHandle* m_pNh;
    ros::Subscriber* m_pVelSub;
    ros::Subscriber* m_pOccupancySub;
    ros::Subscriber* m_pCurGpsSub;
    ros::Subscriber* m_pGoalGpsSub;
    ros::Publisher* m_pVelPub;

    //status
    geometry_msgs::Twist m_curVel;
    sensor_msgs::NavSatFix m_goalGPS;
    sensor_msgs::NavSatFix m_curGPS;



};


#endif //PROJECT_LOCALPLANNER_H
