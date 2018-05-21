//
// Created by tom on 20/05/18.
//

#include "autonomy_master_logic.h"
#include "../../../../../../../../../opt/ros/kinetic/include/sensor_msgs/NavSatFix.h"

#include <string>

CAutonomyMasterLogic::CAutonomyMasterLogic(ros::NodeHandle &nh)
 : m_state(eAutonomyState::IDLE),
   m_pGoalGps(nullptr),
   m_pBacktrackGps(nullptr),
   m_pLocalPlannerStatus(nullptr)
{
  /// Create Subscribers ///
  // Goal Subscriber
  std::string strGoalGpsTopic = "/Autonomy/goal_gps";
  ros::param::get("/AutonomyParams/goalGpsTopic", strGoalGpsTopic);
  m_pGoalGpsSub = new ros::Subscriber(nh.subscribe(strGoalGpsTopic, 1, &CAutonomyMasterLogic::GoalGpsCallback, this));

  // Backtrack Subscriber
  std::string strBacktrackGpsTopic = "/autonomy/backtrack_gps";
  ros::param::get("/AutonomyParams/backtrackGpsTopic", strBacktrackGpsTopic);
  m_pBacktrackGpsSub = new ros::Subscriber(nh.subscribe(strBacktrackGpsTopic, 1, &CAutonomyMasterLogic::BacktrackGpsCallback, this));

  // Local Planner Status Subscriber
  std::string strLocalPlannerStatusTopic = "/local_planner/status";
  ros::param::get("/local_planner/status_topic", strLocalPlannerStatusTopic);
  m_pLocalPlanStatusSub = new ros::Subscriber(nh.subscribe(strLocalPlannerStatusTopic, 1, &CAutonomyMasterLogic::LocalPlannerStatusCallback, this));

  /// Create Publishers ///
  // Local Planner target Publisher
  std::string strLocalPlannerTargetTopic = "/local_planner/goal_gps";
  ros::param::get("/local_planner/goal_gps_topic", strLocalPlannerTargetTopic);
  m_pTargetGpsPub = new ros::Publisher(nh.advertise<sensor_msgs::NavSatFix>(strLocalPlannerTargetTopic, 1));



  //TODO: tie in twist mux


}


  ////////////////////////////
 /// Subscriber Callbacks ///
////////////////////////////

void CAutonomyMasterLogic::GoalGpsCallback(sensor_msgs::NavSatFixConstPtr pGoalGps){
  m_pGoalGps = pGoalGps;
  if (m_state == eAutonomyState::LOCALPLAN) {
    m_pTargetGpsPub->publish(m_pGoalGps);
  }
}

void CAutonomyMasterLogic::BacktrackGpsCallback(sensor_msgs::NavSatFixConstPtr pBacktrackGps) {
  m_pBacktrackGps = pBacktrackGps;
  if (m_state == eAutonomyState::BACKTRACK) {
    m_pTargetGpsPub->publish(m_pBacktrackGps);
  }
}

void CAutonomyMasterLogic::LocalPlannerStatusCallback(local_planner::LocalPlannerStatusConstPtr pLocalPlannerStatus) {
  m_pLocalPlannerStatus = pLocalPlannerStatus;
}




  //////////////
 /// Helpers ///
///////////////

void CAutonomyMasterLogic::UpdateState() {
  switch (m_state)
  {
    case eAutonomyState::LOCALPLAN:
      //TODO: if tennisballdetected-->follow
      if (m_pBacktrackGps && TimeSinceMessage(m_pBacktrackGps->header.stamp) < 2.0) //TODO: softcode time
      {
        StateTransition(eAutonomyState::BACKTRACK);
      } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
        StateTransition(eAutonomyState::TENNISBALL_SEARCH);
      }
      break;
    case eAutonomyState::BACKTRACK:
      //TODO: if tennisballdetected-->follow
      if (m_pBacktrackGps && TimeSinceMessage(m_pBacktrackGps->header.stamp) > 2.0)
      {
        //switch back to local planner
        StateTransition(eAutonomyState::LOCALPLAN);
      } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
        //reached old location but still no service -- what to do?
      }
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      //TODO: if tennisballdetected--follow

      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      //TODO: if tennisballReached-->idle
      break;
    case eAutonomyState::IDLE:
      if (m_pGoalGps && TimeSinceMessage(m_pGoalGps->header.stamp)  < 10.0) {
        StateTransition(eAutonomyState::LOCALPLAN);
      }
      break;
    default:
      //This shouldn't happen, maybe force it back to local planner?
      break;
  }
}

void CAutonomyMasterLogic::StateTransition(eAutonomyState newState) {
  m_state = newState;
  switch (newState)
  {
    case eAutonomyState::LOCALPLAN:
      if (m_pGoalGps)
      {
        m_pTargetGpsPub->publish(m_pGoalGps);
      }
      break;
    case eAutonomyState::BACKTRACK:
      if (m_pBacktrackGps)
      {
        m_pTargetGpsPub->publish(m_pBacktrackGps);
      }
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      //TODO: start process
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      //TODO: start process

      break;
    case eAutonomyState::IDLE:
      m_pGoalGps = nullptr;
      m_pBacktrackGps = nullptr;
      m_pLocalPlannerStatus = nullptr;
      //TODO: endProcess
      break;
    default:
      //This shouldn't happen, maybe force it back to local planner?
      break;
  }
}

void CAutonomyMasterLogic::RunState() {

}


 /////////////////
/// Main Loop ///
////////////////

void CAutonomyMasterLogic::Start() {
  while (ros::ok()) {
    ros::spinOnce();
    UpdateState();
    RunState();
    //twistmux
  }

}

