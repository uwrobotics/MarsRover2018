//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_H
#define PROJECT_AUTONOMY_MASTER_H
#include <ros/ros.h>

typedef enum {
  LOCALPLAN,
  BACKTRACK,
  TENNISBALL_SEARCH,
  TENNISBALL_FOLLOW,
  IDLE
} eAutonomyState;

static inline double TimeSinceMessage(const ros::Time &time) {
  return (ros::Time::now() - time).toSec();
}

#endif // PROJECT_AUTONOMY_MASTER_H
