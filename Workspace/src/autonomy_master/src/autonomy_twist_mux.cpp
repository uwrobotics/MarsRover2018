//
// Created by tom on 21/05/18.
//

#include "autonomy_twist_mux.h"
#define EXPIRY_TIME (3.0)

CAutonomyTwistMux::CAutonomyTwistMux(ros::NodeHandle &nh)
 : m_pBallFallowVel(nullptr), m_pSpiralVel(nullptr), m_pLocalPlannerVel(nullptr), m_pUnstuckVel(nullptr), m_pEStopVel(nullptr)
{
  // Create publisher
  m_pCmdVelPub = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));

  /// Create Subscribers ///
  // EStop Subscriber
  std::string strEStopTopic = "/estop/cmd_vel";
  ros::param::get("/AutonomyParams/estop_topic", strEStopTopic);
  m_pEStopSub = new ros::Subscriber(nh.subscribe(strEStopTopic, 1, &CAutonomyTwistMux::EStopCallback, this));

  // LocalPlanner Subscriber
  std::string strLocalPlannerTopic = "/local_planner/cmd_vel";
  ros::param::get("/AutonomyParams/local_plan_vel_topic", strLocalPlannerTopic);
  m_pLocalPlannerSub = new ros::Subscriber(nh.subscribe(strLocalPlannerTopic, 1, &CAutonomyTwistMux::LocalPlannerCallback, this));

  // Unstuck Subscriber
  std::string strUnstuckTopic = "/unstuck/cmd_vel";
  ros::param::get("/AutonomyParams/unstuck_topic", strUnstuckTopic);
  m_pUnstuckSub = new ros::Subscriber(nh.subscribe(strUnstuckTopic, 1, &CAutonomyTwistMux::UnstuckCallback, this));

  // Spiral Subscriber
  std::string strSpiralTopic = "/spiral/cmd_vel";
  ros::param::get("/AutonomyParams/spiral_topic", strSpiralTopic);
  m_pSpiralSub = new ros::Subscriber(nh.subscribe(strSpiralTopic, 1, &CAutonomyTwistMux::SpiralCallback, this));

  // BallFollower Subscriber
  std::string strBallFollowerTopic = "/ball_follower/cmd_vel";
  ros::param::get("/AutonomyParams/ball_follow_topic", strBallFollowerTopic);
  m_pBallFolowSub = new ros::Subscriber(nh.subscribe(strBallFollowerTopic, 1, &CAutonomyTwistMux::BallFollowCallback, this));


}

void CAutonomyTwistMux::Arbitrate(eAutonomyState state) {
  geometry_msgs::Twist chosenVel;
  switch (state)
  {
    case eAutonomyState::LOCALPLAN:
    case eAutonomyState::BACKTRACK:
      if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pEStopVel;
      } else if (m_pUnstuckVel && (TimeSinceMessage(m_UnstuckRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pUnstuckVel;
      } else if (m_pLocalPlannerVel && (TimeSinceMessage(m_LocalPlanRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pLocalPlannerVel;
      }
      break;
    case eAutonomyState::TENNISBALL_SEARCH:
      if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pEStopVel;
      } else if (m_pSpiralVel && (TimeSinceMessage(m_SpiralRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pSpiralVel;
      }
      break;
    case eAutonomyState::TENNISBALL_FOLLOW:
      if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pEStopVel;
      } else if (m_pBallFallowVel && (TimeSinceMessage(m_BallFollowRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pBallFallowVel;
      }
      break;
    case eAutonomyState::IDLE:
      if (m_pEStopVel && (TimeSinceMessage(m_EStopRecTime) < EXPIRY_TIME)) {
        chosenVel = *m_pEStopVel;
      }
      break;
    default:
      break;
  }
  m_pCmdVelPub->publish(chosenVel);

}

void CAutonomyTwistMux::LocalPlannerCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pLocalPlannerVel = pVel;
  m_LocalPlanRecTime = ros::Time::now();
}

void CAutonomyTwistMux::UnstuckCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pUnstuckVel = pVel;
  m_UnstuckRecTime = ros::Time::now();
}

void CAutonomyTwistMux::EStopCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pEStopVel = pVel;
  m_EStopRecTime = ros::Time::now();
}

void CAutonomyTwistMux::SpiralCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pSpiralVel = pVel;
  m_SpiralRecTime = ros::Time::now();
}

void CAutonomyTwistMux::BallFollowCallback(geometry_msgs::TwistConstPtr pVel) {
  m_pBallFallowVel = pVel;
  m_BallFollowRecTime = ros::Time::now();
}
