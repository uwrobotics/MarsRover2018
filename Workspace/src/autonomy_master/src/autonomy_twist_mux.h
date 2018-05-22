//
// Created by tom on 21/05/18.
//

#ifndef PROJECT_AUTONOMY_TWIST_MUX_H
#define PROJECT_AUTONOMY_TWIST_MUX_H
#include "autonomy_master.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class CAutonomyTwistMux {
public:
  CAutonomyTwistMux(ros::NodeHandle& nh);
  void Arbitrate(eAutonomyState state);

private:
  /// Subscriber callbacks ///


  /// Subscribers ///




  /// Publishers ///



  /// Received twists ///

};


#endif //PROJECT_AUTONOMY_TWIST_MUX_H
