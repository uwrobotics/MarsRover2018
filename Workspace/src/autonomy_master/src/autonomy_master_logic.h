//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_LOGIC_H
#define PROJECT_AUTONOMY_MASTER_LOGIC_H

#include "autonomy_master.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "local_planner/LocalPlannerStatus.h"


class CAutonomyMasterLogic {
public:
  CAutonomyMasterLogic(ros::NodeHandle& nh);
  //~CAutonomyMasterLogic();
  void Start();


private:
  void UpdateState();
  void StateTransition(eAutonomyState newState);
  void RunState();

  /// Subscriber Callbacks ///
  void GoalGpsCallback(sensor_msgs::NavSatFixConstPtr pGoalGps);
  void BacktrackGpsCallback(sensor_msgs::NavSatFixConstPtr pBacktrackGps);
  void LocalPlannerStatusCallback(local_planner::LocalPlannerStatusConstPtr pLocalPlannerStatus);

  eAutonomyState m_state;

  /// Subscribers ///
  ros::Subscriber* m_pGoalGpsSub;
  ros::Subscriber* m_pBacktrackGpsSub;
  ros::Subscriber* m_pLocalPlanStatusSub;

  /// Publishers ///
  ros::Publisher* m_pTargetGpsPub;
  //TODO: tennisball interface

  /// Received messages ///
  sensor_msgs::NavSatFixConstPtr m_pGoalGps;
  sensor_msgs::NavSatFixConstPtr m_pBacktrackGps;
  local_planner::LocalPlannerStatusConstPtr m_pLocalPlannerStatus;
  //TODO: tennis ball interface



};


#endif //PROJECT_AUTONOMY_MASTER_LOGIC_H
