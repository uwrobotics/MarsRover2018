//
// Created by tom on 20/05/18.
//

#include "autonomy_master_logic.h"
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

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

  // Ball Detection Subscriber
  std::string strBallDetectionTopic = "/Autonomy/ball_tracker/ball_detection";
  ros::param::get("/autonomy/detection_topic", strBallDetectionTopic);
  m_pBallDetectionSub = new ros::Subscriber(nh.subscribe(strBallDetectionTopic, 1, &CAutonomyMasterLogic::BallDetectionCallback, this));

  // Ball follower Subscriber
  std::string strBallFollowerTopic = "/Autonomy/ball_follower/ball_reached";
  ros::param::get("/autonomy/follower_topic", strBallFollowerTopic);
  m_pBallFollowerSub = new ros::Subscriber(nh.subscribe(strBallFollowerTopic, 1, &CAutonomyMasterLogic::BallFollowerCallback, this));

  /// Create Publishers ///
  // Local Planner target Publisher
  std::string strLocalPlannerTargetTopic = "/local_planner/goal_gps";
  ros::param::get("/local_planner/goal_gps_topic", strLocalPlannerTargetTopic);
  m_pTargetGpsPub = new ros::Publisher(nh.advertise<sensor_msgs::NavSatFix>(strLocalPlannerTargetTopic, 1));

  // BallFollower Enable Publisher
  std::string strBallFollowerEnableTopic = "/ball_follower/enable";
  ros::param::get("/autonomy/ball_follower_enable_topic", strBallFollowerEnableTopic);
  m_pBallFollowerPub = new ros::Publisher(nh.advertise<std_msgs::Bool>(strBallFollowerEnableTopic, 1));



  //TODO: tie in twist mux
  m_pTwistMux = new CAutonomyTwistMux(nh);



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

void CAutonomyMasterLogic::BallDetectionCallback(ball_tracker::BallDetectionConstPtr pBallDetection) {
  m_bBallDetected=  (pBallDetection->isDetected && pBallDetection->isStable);
}

void CAutonomyMasterLogic::BallFollowerCallback(std_msgs::BoolConstPtr pMsg) {
  m_bBallDetected = pMsg->data;
}


  //////////////
 /// Helpers ///
///////////////

void CAutonomyMasterLogic::UpdateState() {
  switch (m_state)
  {
    case eAutonomyState::LOCALPLAN:
      if (m_bBallDetected) {
        StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
      } else if (m_pBacktrackGps && TimeSinceMessage(m_pBacktrackGps->header.stamp) < 2.0) //TODO: softcode time
      {
        StateTransition(eAutonomyState::BACKTRACK);
      } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
        StateTransition(eAutonomyState::TENNISBALL_SEARCH);
      }
      break;
    case eAutonomyState::BACKTRACK:
      if (m_bBallDetected) {
        StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
      } else if (m_pBacktrackGps && TimeSinceMessage(m_pBacktrackGps->header.stamp) > 2.0)
      {
        //switch back to local planner
        StateTransition(eAutonomyState::LOCALPLAN);
      } else if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalReached) {
        //reached old location but still no service -- what to do?
      }
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      if (m_bBallDetected) {
        StateTransition(eAutonomyState::TENNISBALL_FOLLOW);
      }
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      //TODO: if tennisballReached-->idle
      if (m_bBallReached) {
        StateTransition(eAutonomyState::IDLE);
      }
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
      //TODO: start process, need spiral search interface
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      //TODO: not sure if this is the right spot
      {
        std_msgs::Bool msg;
        msg.data = true;
        m_pBallFollowerPub->publish(msg);
      }
      break;
    case eAutonomyState::IDLE: {
      m_pGoalGps = nullptr;
      m_pBacktrackGps = nullptr;
      m_pLocalPlannerStatus = nullptr;
      m_bBallDetected = false;
      //TODO: endProcess
      std_msgs::Bool msg;
      msg.data = false;
      m_pBallFollowerPub->publish(msg);
    }
      break;
    default:
      //This shouldn't happen, maybe force it back to local planner?
      break;
  }
}

// Do actions that should be done every iteration for the given state
void CAutonomyMasterLogic::RunState() {
  switch (m_state)
  {
    case eAutonomyState::LOCALPLAN:
      //need to check if we're in range to start detecting
      if (m_pLocalPlannerStatus && m_pLocalPlannerStatus->goalInRange)
      {
        // start looking for the ball since we might pass it
        ///std_msgs::Bool msg;
        ///msg.data = true;
        ///m_pBallFollowerPub->publish(msg);
      }
      break;
    case eAutonomyState::BACKTRACK:
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      break;
    case eAutonomyState::IDLE:
      break;
    default:
      //This shouldn't happen, maybe force it back to local planner?
      break;
  }

}


  /////////////////
 /// Main Loop ///
/////////////////

void CAutonomyMasterLogic::Start() {
  while (ros::ok()) {
    ros::spinOnce();
    UpdateState();
    RunState();
    //twistmux
    m_pTwistMux->Arbitrate(m_state);
  }

}



