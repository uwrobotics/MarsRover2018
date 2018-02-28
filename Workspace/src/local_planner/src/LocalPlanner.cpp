//
// Created by tom on 20/02/18.
//

#include "LocalPlanner.h"
#include "robot_localization/navsat_conversions.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>


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
    //m_pVelSub = new ros::Subscriber(
    //        m_pNh->subscribe("current_vel",1,&CLocalPlanner::CurVelCallback, this));
    //m_pCurGpsSub = new ros::Subscriber(
    //        m_pNh->subscribe("cur_gps",1,&CLocalPlanner::CurGPSCallback, this));
    m_pGoalGpsSub = new ros::Subscriber(
            m_pNh->subscribe("/local_plan/goal_gps",1,&CLocalPlanner::GoalGPSCallback, this));
    m_pOdometrySub = new ros::Subscriber(
            m_pNh->subscribe("/odometry/rover_gps_odom",1,&CLocalPlanner::OdometryCallback, this));

    m_pVelPub = new ros::Publisher(m_pNh->advertise<geometry_msgs::Twist>("cmd_vel",1));
}

void CLocalPlanner::CurVelCallback(geometry_msgs::Twist::ConstPtr vel)
{
    m_curVel = *vel;
}

void CLocalPlanner::GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal)
{
    m_goalGPS = *goal;
    RobotLocalization::NavsatConversions::LLtoUTM(goal->latitude,
                                                  goal->longitude,
                                                  m_goalGpsUtmX,
                                                  m_goalGpsUtmY,
                                                  m_goalGpsUtmZone);

}

void CLocalPlanner::CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps)
{
    m_curGPS = *gps;
}

void CLocalPlanner::OdometryCallback(nav_msgs::Odometry::ConstPtr odometry)
{

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped roverLocToUTM;
    try{
        roverLocToUTM = tfBuffer.lookupTransform("utm", odometry->child_frame_id, ros::Time(0), ros::Duration(2));
    }
    catch (tf2::TransformException& ex ){
        ROS_ERROR("%s",ex.what());
    }

    ROS_INFO("Generated transform from base_link to utm");
    m_curGpsUtmX = roverLocToUTM.transform.translation.x;
    m_curGpsUtmY = roverLocToUTM.transform.translation.y;


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
    heading = yaw;

    m_orientationToGoal = atan2(-(m_goalGpsUtmX - m_curGpsUtmX),
                                m_goalGpsUtmY - m_curGpsUtmY) - heading;
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


    CDynamicWindow dynamicWindow(m_curVel.linear.x,m_curVel.angular.z,m_robotParams);

    geometry_msgs::Twist chosenVel = dynamicWindow.AssessOccupancyGrid(grid,m_orientationToGoal);

    m_pVelPub->publish(chosenVel);
}