//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_LOGIC_H
#define PROJECT_AUTONOMY_MASTER_LOGIC_H

#include "autonomy_master.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include "local_planner/LocalPlannerStatus.h"
#include "ball_tracker/BallDetection.h"
#include "autonomy_twist_mux.h"


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
  void BallDetectionCallback(ball_tracker::BallDetectionConstPtr pBallDetection);
  void BallFollowerCallback(std_msgs::BoolConstPtr pMsg);

  eAutonomyState m_state;
  bool m_bBallDetected;
  bool m_bBallReached;


  /// Subscribers ///
  ros::Subscriber* m_pGoalGpsSub;
  ros::Subscriber* m_pBacktrackGpsSub;
  ros::Subscriber* m_pLocalPlanStatusSub;
  ros::Subscriber* m_pBallDetectionSub;
  ros::Subscriber* m_pBallFollowerSub;

  /// Publishers ///
  ros::Publisher* m_pTargetGpsPub;
  //TODO: tennisball interface
  ros::Publisher* m_pBallFollowerPub;

  /// Received messages ///
  sensor_msgs::NavSatFixConstPtr m_pGoalGps;
  sensor_msgs::NavSatFixConstPtr m_pBacktrackGps;
  local_planner::LocalPlannerStatusConstPtr m_pLocalPlannerStatus;
  //TODO: tennis ball interface

  /// Twist Mux ///
  CAutonomyTwistMux* m_pTwistMux;

};


#endif //PROJECT_AUTONOMY_MASTER_LOGIC_H
