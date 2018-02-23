//
// Created by tom on 20/02/18.
//

#include "LocalPlanner.h"

CLocalPlanner::CLocalPlanner(ros::NodeHandle *pNh, const RobotParams_t& robotParams)
 : m_pNh(pNh),
   m_pOccupancySub(nullptr),
   m_pGoalGpsSub(nullptr),
   m_pCurGpsSub(nullptr),
   m_pVelSub(nullptr),
   m_pVelPub(nullptr),
   m_robotParams(robotParams)
{
    m_pOccupancySub =  new ros::Subscriber(
            m_pNh->subscribe("occupancy_grid",1,&CLocalPlanner::OccupancyCallback, this));
    m_pVelSub = new ros::Subscriber(
            m_pNh->subscribe("current_vel",1,&CLocalPlanner::CurVelCallback, this));
    m_pCurGpsSub = new ros::Subscriber(
            m_pNh->subscribe("cur_gps",1,&CLocalPlanner::CurGPSCallback, this));
    m_pGoalGpsSub = new ros::Subscriber(
            m_pNh->subscribe("goal_gps",1,&CLocalPlanner::GoalGPSCallback, this));
    m_pVelPub = new ros::Publisher(m_pNh->advertise<geometry_msgs::Twist>("local_planner_vel",1));
}

void CLocalPlanner::CurVelCallback(geometry_msgs::Twist::ConstPtr vel)
{
    m_curVel = *vel;
}

void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal)
{
    m_goalGPS = *goal;
}

void CLocalPlanner::CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps)
{
    m_curGPS = *gps;
}

/*
///Occupancy Callback:
1) blur the occupancy grid if needed
2) find slope if needed
3) construct dynamic window based on cur vel
4) assess distances
5) compute scores
6) select bests

 */
void CLocalPlanner::OccupancyCallback(local_planner::OccupancyGrid::ConstPtr grid)
{

}