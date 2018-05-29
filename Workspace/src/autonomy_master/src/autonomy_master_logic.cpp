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
   m_pLocalPlannerStatus(nullptr),
   m_bBallReached(false),
   m_bBallDetected(false),
   m_bBallLost(false)
{
  /// Create Subscribers ///
  // Goal Subscriber
  std::string strGoalGpsTopic = "/autonomy/goal_gps";
  ros::param::get("/autonomy/goal_gps_topic", strGoalGpsTopic);
  m_pGoalGpsSub = new ros::Subscriber(nh.subscribe(strGoalGpsTopic, 1, &CAutonomyMasterLogic::GoalGpsCallback, this));

  // Backtrack Subscriber
  std::string strBacktrackGpsTopic = "/backtrack_ping/backtrack_gps";
  ros::param::get("/autonomy/backtrack_gps_topic", strBacktrackGpsTopic);
  m_pBacktrackGpsSub = new ros::Subscriber(nh.subscribe(strBacktrackGpsTopic, 1, &CAutonomyMasterLogic::BacktrackGpsCallback, this));

  // Local Planner Status Subscriber
  std::string strLocalPlannerStatusTopic = "/local_planner/status";
  ros::param::get("/autonomy/local_plan_status_topic", strLocalPlannerStatusTopic);
  m_pLocalPlanStatusSub = new ros::Subscriber(nh.subscribe(strLocalPlannerStatusTopic, 1, &CAutonomyMasterLogic::LocalPlannerStatusCallback, this));

  // Ball Detection Subscriber
  std::string strBallDetectionTopic = "/ball_tracker/detection";
  ros::param::get("/autonomy/ball_detection_topic", strBallDetectionTopic);
  m_pBallDetectionSub = new ros::Subscriber(nh.subscribe(strBallDetectionTopic, 1, &CAutonomyMasterLogic::BallDetectionCallback, this));

  // Ball follower Arrived Subscriber
  std::string strBallFollowerArrivedTopic = "/ball_following/arrival";
  ros::param::get("/autonomy/ball_follower_arrived_topic", strBallFollowerArrivedTopic);
  m_pBallFollowerArrivedSub = new ros::Subscriber(nh.subscribe(strBallFollowerArrivedTopic, 1, &CAutonomyMasterLogic::BallFollowerArrivedCallback, this));

  // Ball follower Lost Subscriber
  std::string strBallFollowerLostTopic = "/ball_tracker/lost_ball";
  ros::param::get("/autonomy/ball_tracker_lost_topic", strBallFollowerLostTopic);
  m_pBallFollowerLostSub = new ros::Subscriber(nh.subscribe(strBallFollowerLostTopic, 1, &CAutonomyMasterLogic::BallFollowerLostCallback, this));

  /// Create Publishers ///
  // Local Planner target Publisher
  std::string strLocalPlannerTargetTopic = "/local_planner/goal_gps";
  ros::param::get("/autonomy/target_gps_topic", strLocalPlannerTargetTopic);
  m_pTargetGpsPub = new ros::Publisher(nh.advertise<sensor_msgs::NavSatFix>(strLocalPlannerTargetTopic, 1));

  // LocalPlanner Enable Publisher
  std::string strLocalPlannerEnableTopic = "/local_planner/enable";
  ros::param::get("/autonomy/local_planner_enable_topic", strLocalPlannerEnableTopic);
  m_pLocalPlannerEnablePub = new ros::Publisher(nh.advertise<std_msgs::Bool>(strLocalPlannerEnableTopic, 1));

  // Spiral Enable Publisher
  std::string strSpiralEnableTopic = "/spiral/enable";
  ros::param::get("/autonomy/spiral_enable_topic", strSpiralEnableTopic);
  m_pSpiralEnablePub = new ros::Publisher(nh.advertise<std_msgs::Bool>(strSpiralEnableTopic, 1));

  // BallTracker Enable Publisher
  std::string strBallTrackerEnableTopic = "/ball_tracker/enable";
  ros::param::get("/autonomy/ball_tracker_enable_topic", strBallTrackerEnableTopic);
  m_pBallTrackerPub = new ros::Publisher(nh.advertise<std_msgs::Bool>(strBallTrackerEnableTopic, 1));



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
  m_bBallDetected =  (pBallDetection->isDetected && pBallDetection->isStable);
}

void CAutonomyMasterLogic::BallFollowerArrivedCallback(std_msgs::EmptyConstPtr pMsg){
  m_bBallReached = true;
}

void CAutonomyMasterLogic::BallFollowerLostCallback(std_msgs::EmptyConstPtr pMsg) {
  m_bBallLost = true;
}

  ///////////////
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
      } else if (m_bBallLost) {
        StateTransition(eAutonomyState::TENNISBALL_SEARCH);
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
      ROS_INFO("Entering state: Local Planner");
      if (m_pGoalGps)
      {
        m_pTargetGpsPub->publish(m_pGoalGps);
        std_msgs::Bool enable;
        enable.data=true;
        m_pLocalPlannerEnablePub->publish(enable);
      }
      break;
    case eAutonomyState::BACKTRACK:
      ROS_INFO("Entering state: Backtrack");
      if (m_pBacktrackGps)
      {
        m_pTargetGpsPub->publish(m_pBacktrackGps);
        std_msgs::Bool enable;
        enable.data=true;
        m_pLocalPlannerEnablePub->publish(enable);
      }
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      ROS_INFO("Entering state: Ball Search");
      //TODO: start process, need spiral search interface
      {
        m_bBallDetected = false;
        m_bBallLost = false;
        m_bBallReached = false;

        std_msgs::Bool enable;
        enable.data=false;
        m_pSpiralEnablePub->publish(enable);
        m_pBallTrackerPub->publish(enable);

        std_msgs::Bool disable;
        disable.data=false;
        m_pLocalPlannerEnablePub->publish(disable);
      }
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      ROS_INFO("Entering state: Ball Follow");
      //TODO: not sure if this is the right spot
      {
        m_bBallLost = false;
        m_bBallReached = false;

        std_msgs::Bool enable;
        enable.data = true;
        m_pBallTrackerPub->publish(enable);

        std_msgs::Bool disable;
        disable.data=false;
        m_pLocalPlannerEnablePub->publish(disable);
        m_pSpiralEnablePub->publish(disable);
      }
      break;
    case eAutonomyState::IDLE: {
      ROS_INFO("Entering state: Idle");
      m_pGoalGps = nullptr;
      m_pBacktrackGps = nullptr;
      m_pLocalPlannerStatus = nullptr;
      m_bBallDetected = false;
      m_bBallReached = false;
      m_bBallLost = false;
      //TODO: endProcess
      std_msgs::Bool disable;
      disable.data = false;
      m_pBallTrackerPub->publish(disable);
      m_pLocalPlannerEnablePub->publish(disable);
      m_pSpiralEnablePub->publish(disable);
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
        std_msgs::Bool msg;
        msg.data = true;
        m_pBallTrackerPub->publish(msg);
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



