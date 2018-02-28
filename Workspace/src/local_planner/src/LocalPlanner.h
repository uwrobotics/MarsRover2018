//
// Created by tom on 20/02/18.
//

#ifndef PROJECT_LOCALPLANNER_H
#define PROJECT_LOCALPLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "occupancy_grid/OccupancyGrid.h"
#include "DynamicWindow.h"

class CLocalPlanner
{
public:
    explicit CLocalPlanner(ros::NodeHandle* pNh, const RobotParams_t& robotParams);

private:
    //Subscriber callbacks
    void CurVelCallback(geometry_msgs::Twist::ConstPtr vel);
    void GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal);
    void CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps);
    void OccupancyCallback(occupancy_grid::OccupancyGrid::ConstPtr grid);
    void OdometryCallback(nav_msgs::Odometry::ConstPtr odemetry);


    //Ros handlers
    ros::NodeHandle* m_pNh;
    ros::Subscriber* m_pVelSub;
    ros::Subscriber* m_pOccupancySub;
    ros::Subscriber* m_pCurGpsSub;
    ros::Subscriber* m_pGoalGpsSub;
    ros::Subscriber* m_pOdometrySub;
    ros::Publisher* m_pVelPub;

    //status
    geometry_msgs::Twist m_curVel;
    sensor_msgs::NavSatFix m_goalGPS;
    sensor_msgs::NavSatFix m_curGPS;
    double m_curGpsUtmX;
    double m_curGpsUtmY;
    std::string m_curGpsUtmZone;
    double m_goalGpsUtmX;
    double m_goalGpsUtmY;
    std::string m_goalGpsUtmZone;
    double m_orientationToGoal;
    //Parameters
    const RobotParams_t& m_robotParams;



};


#endif //PROJECT_LOCALPLANNER_H
