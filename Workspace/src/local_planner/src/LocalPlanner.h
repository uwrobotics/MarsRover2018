//
// Created by tom on 20/02/18.
//

#ifndef PROJECT_LOCALPLANNER_H
#define PROJECT_LOCALPLANNER_H

#include "DynamicWindow.h"
#include "occupancy_grid/OccupancyGrid.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/Point.h>
#include <mutex>
#include <thread>

class CLocalPlanner {
public:
  explicit CLocalPlanner(ros::NodeHandle *pNh,
                         const RobotParams_t &robotParams);

private:
  // Subscriber callbacks
  void CurVelCallback(geometry_msgs::Twist::ConstPtr vel);
  // void GoalGPSCallback(sensor_msgs::NavSatFix::ConstPtr goal);
  void GoalGPSCallback(geometry_msgs::Point::ConstPtr goal);
  void CurGPSCallback(sensor_msgs::NavSatFix::ConstPtr gps);
  void OccupancyCallback(occupancy_grid::OccupancyGrid::ConstPtr grid);
  void OdometryCallback(nav_msgs::Odometry::ConstPtr odemetry);
  void VelocityPublisher();

  // Ros handlers
  ros::NodeHandle *m_pNh;
  ros::Subscriber *m_pVelSub;
  ros::Subscriber *m_pOccupancySub;
  ros::Subscriber *m_pCurGpsSub;
  ros::Subscriber *m_pGoalGpsSub;
  ros::Subscriber *m_pOdometrySub;
  ros::Publisher *m_pVelPub;

  // status
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
  // Parameters
  const RobotParams_t &m_robotParams;

  bool m_bOdomReceived;
  bool m_bGoalReceived;
  bool m_bGoalReached;

  // velocity publishing
  std::mutex m_velMutex;
  geometry_msgs::Twist m_targetVel;
  bool m_bVelocityReady;
  std::thread *m_pVelPubThread;

  // tracking lost obstacles
  double m_distanceSinceLastRightDanger;
  double m_distanceSinceLastLeftDanger;
  double m_lastDwaCoordUtmX;
  double m_lastDwaCoordUtmY;
};

#endif // PROJECT_LOCALPLANNER_H
