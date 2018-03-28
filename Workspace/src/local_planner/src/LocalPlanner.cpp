//
// Created by tom on 20/02/18.
//

#include "LocalPlanner.h"
#include "robot_localization/navsat_conversions.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <cstdio>

#include <robot_localization/navsat_conversions.h>
//#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
//#include <rover_autonomy/gps_coord.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32MultiArray.h>

#define IGNORE_DANGER_THRESHOLD 0.50

CLocalPlanner::CLocalPlanner(ros::NodeHandle *pNh, const RobotParams_t& robotParams)
 : m_pNh(pNh),
   m_pOccupancySub(nullptr),
   m_pGoalGpsSub(nullptr),
   m_pCurGpsSub(nullptr),
   m_pVelSub(nullptr),
   m_pVelPub(nullptr),
   m_robotParams(robotParams),
   m_bOdomReceived(false),
   m_bGoalReceived(false),
   m_bVelocityReady(true),
   m_pVelPubThread(nullptr),
   m_distanceSinceLastRightDanger(1000),
   m_distanceSinceLastLeftDanger(1000)
{
    m_pOccupancySub =  new ros::Subscriber(
            m_pNh->subscribe("/OccupancyGrid",1,&CLocalPlanner::OccupancyCallback, this));
    //m_pVelSub = new ros::Subscriber(
    //        m_pNh->subscribe("current_vel",1,&CLocalPlanner::CurVelCallback, this));
    //m_pCurGpsSub = new ros::Subscriber(
    //        m_pNh->subscribe("cur_gps",1,&CLocalPlanner::CurGPSCallback, this));
    m_pGoalGpsSub = new ros::Subscriber(
            m_pNh->subscribe("/local_plan/goal_gps",1,&CLocalPlanner::GoalGPSCallback, this));
    m_pOdometrySub = new ros::Subscriber(
            m_pNh->subscribe("/odometry/filtered",1,&CLocalPlanner::OdometryCallback, this));

    m_pVelPub = new ros::Publisher(m_pNh->advertise<geometry_msgs::Twist>("cmd_vel",1));

    m_pVelPubThread =  new std::thread(&CLocalPlanner::VelocityPublisher, this);

}

void CLocalPlanner::CurVelCallback(geometry_msgs::Twist::ConstPtr vel)
{
    m_curVel = *vel;
}

//void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal)
void CLocalPlanner::GoalGPSCallback(geometry_msgs::Point::ConstPtr goal)
{
//    m_goalGPS = *goal;
//    RobotLocalization::NavsatConversions::LLtoUTM(goal->latitude,
 //                                                 goal->longitude,
 //                                                 m_goalGpsUtmX,
 //                                                 m_goalGpsUtmY,
 //                                                 m_goalGpsUtmZone);

//FOR SAR ONLY: use position in "map" frame
    m_goalGpsUtmX = goal->x;
    m_goalGpsUtmY = goal->y;


    m_bGoalReceived = true;

    ROS_INFO("New goal: x=%f, y=%f",m_goalGpsUtmX, m_goalGpsUtmY);
}

void CLocalPlanner::CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps)
{
    m_curGPS = *gps;
}

void CLocalPlanner::OdometryCallback(nav_msgs::Odometry::ConstPtr odometry)
{

//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    geometry_msgs::TransformStamped roverLocToUTM;
//    try{
//        roverLocToUTM = tfBuffer.lookupTransform("utm", /*odometry->child_frame_id*/odometry->header.frame_id, ros::Time(0), ros::Duration(2));
//    }
//    catch (tf2::TransformException& ex ){
//        ROS_ERROR("%s",ex.what());
//    }

    //ROS_INFO("Generated transform from base_link to utm");
//    m_curGpsUtmX = roverLocToUTM.transform.translation.x;
//    m_curGpsUtmY = roverLocToUTM.transform.translation.y;

//FOR SAR ONLY: use position on map, figure out utm later
    m_curGpsUtmX = odometry->pose.pose.position.x;
    m_curGpsUtmY = odometry->pose.pose.position.y;

    m_curVel = odometry->twist.twist;

    double heading = 0;////TODO
    tf::Quaternion q(
            odometry->pose.pose.orientation.x,
            odometry->pose.pose.orientation.y,
            odometry->pose.pose.orientation.z,
            odometry->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    heading = yaw - M_PI/2;
if (heading < -M_PI)
{
    heading += 2*M_PI; 
}

    double globOrient = atan2(-(m_goalGpsUtmX - m_curGpsUtmX),
                                m_goalGpsUtmY - m_curGpsUtmY);
	m_orientationToGoal = globOrient - heading;



    m_bOdomReceived = true;
static int count=0;
if (count==0){
    ROS_INFO("Current velocity: v=%f, w=%f",m_curVel.linear.x, m_curVel.angular.z);
    ROS_INFO("Current pos: x=%f, y=%f, heading=%f",m_curGpsUtmX, m_curGpsUtmY, heading);

	ROS_INFO("globalOrientToGoal: %f, heading: %f, orientToGoal: %f",globOrient, heading, m_orientationToGoal);
}
count = (count+1)%100;
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
void CLocalPlanner::OccupancyCallback(occupancy_grid::OccupancyGrid::ConstPtr grid)
{
    if (!m_bOdomReceived || !m_bGoalReceived)
    {
        ROS_WARN("ignoring occupancy: not ready");
        return;
    }
    if ((m_curGpsUtmY-m_goalGpsUtmY)*(m_curGpsUtmY-m_goalGpsUtmY) +
                (m_curGpsUtmX-m_goalGpsUtmX)*(m_curGpsUtmX-m_goalGpsUtmX) < 1)
    {
        ROS_INFO("Goal reached\n");
        return;
    }
    ROS_INFO("Received an occupancy grid");
    CDynamicWindow dynamicWindow(m_curVel.linear.x,m_curVel.angular.z,m_robotParams, (m_distanceSinceLastLeftDanger < IGNORE_DANGER_THRESHOLD),
                                                                                     (m_distanceSinceLastRightDanger < IGNORE_DANGER_THRESHOLD));
    ROS_INFO("Created dynamic window");
    geometry_msgs::Twist chosenVel = dynamicWindow.AssessOccupancyGrid(grid,m_orientationToGoal);

    //Check for any danger conditions
    double distSinceLastCheck = sqrt( (m_curGpsUtmX - m_lastDwaCoordUtmX)*(m_curGpsUtmX - m_lastDwaCoordUtmX) +
                                        (m_curGpsUtmY - m_lastDwaCoordUtmY)*(m_curGpsUtmY - m_lastDwaCoordUtmY) );
    if (dynamicWindow.FoundDangerOnRight())
    {
        m_distanceSinceLastRightDanger = 0;
        ROS_INFO("There was danger on right");
    }
    else
    {
        m_distanceSinceLastRightDanger += distSinceLastCheck;
    }
    if (dynamicWindow.FoundDangerOnLeft())
    {
        m_distanceSinceLastLeftDanger = 0;
        ROS_INFO("there was danger on left");
    }
    else
    {
        m_distanceSinceLastLeftDanger += distSinceLastCheck;
    }
    ROS_INFO("Distance since danger: left: %f, right %f", m_distanceSinceLastLeftDanger, m_distanceSinceLastRightDanger);
    m_lastDwaCoordUtmX = m_curGpsUtmX;
    m_lastDwaCoordUtmY = m_curGpsUtmY;

    ROS_INFO("chose v=%f, w=%f", chosenVel.linear.x, chosenVel.angular.z);

    std::unique_lock<std::mutex> lock(m_velMutex);
    m_targetVel = chosenVel;
    m_bVelocityReady = true;
    lock.unlock();
}


void CLocalPlanner::VelocityPublisher()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(m_velMutex);
        if (m_bVelocityReady)
        {
            geometry_msgs::Twist vel = m_targetVel;
            lock.unlock();
            m_pVelPub->publish(vel);
        }
        else
        {
            lock.unlock();
        }
        rate.sleep();
    }
}
