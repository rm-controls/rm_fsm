//
// Created by kiana on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include "rm_fsm/fsm_data.h"

class PowerLimit {
 public:
  PowerLimit(ros::NodeHandle &nh);
  void input(referee::RefereeData referee, const ros::Duration &period);
  double output();

 private:
  control_toolbox::Pid pid_buffer_;

};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
