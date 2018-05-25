//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_LOGIC_H
#define PROJECT_AUTONOMY_MASTER_LOGIC_H

#include "autonomy_master.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
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
  void BallFollowerArrivedCallback(std_msgs::EmptyConstPtr pMsg);
  void BallFollowerLostCallback(std_msgs::EmptyConstPtr pMsg);

  eAutonomyState m_state;
  bool m_bBallDetected;
  bool m_bBallReached;
  bool m_bBallLost;


  /// Subscribers ///
  ros::Subscriber* m_pGoalGpsSub;
  ros::Subscriber* m_pBacktrackGpsSub;
  ros::Subscriber* m_pLocalPlanStatusSub;
  ros::Subscriber* m_pBallDetectionSub;
  ros::Subscriber* m_pBallFollowerArrivedSub;
  ros::Subscriber* m_pBallFollowerLostSub;

  /// Publishers ///
  ros::Publisher* m_pTargetGpsPub;
  ros::Publisher* m_pLocalPlannerEnablePub;
  //TODO: tennisball interface
  ros::Publisher* m_pSpiralEnablePub;
  ros::Publisher* m_pBallTrackerPub;

  /// Received messages ///
  sensor_msgs::NavSatFixConstPtr m_pGoalGps;
  sensor_msgs::NavSatFixConstPtr m_pBacktrackGps;
  local_planner::LocalPlannerStatusConstPtr m_pLocalPlannerStatus;
  //TODO: tennis ball interface

  /// Twist Mux ///
  CAutonomyTwistMux* m_pTwistMux;

};


#endif //PROJECT_AUTONOMY_MASTER_LOGIC_H
