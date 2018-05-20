//
// Created by tom on 20/05/18.
//

#ifndef PROJECT_AUTONOMY_MASTER_LOGIC_H
#define PROJECT_AUTONOMY_MASTER_LOGIC_H

#include "autonomy_master.h"
#include <ros/ros.h>
#include <thread>

class CAutonomyMasterLogic {
public:
  CAutonomyMasterLogic(ros::NodeHandle& nh);
  ~CAutonomyMasterLogic();
  void Start();


private:
  void MainLogicThread();



  std::thread m_logicThread;
};


#endif //PROJECT_AUTONOMY_MASTER_LOGIC_H
