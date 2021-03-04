//
// Created by kiana on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <rm_fsm/referee.h>

class PowerLimit {
 public:
  explicit PowerLimit(ros::NodeHandle &nh);
  void input(referee::RefereeData referee);
  double output();

 private:
  double current_ = 99;
  double danger_surplus_;
};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
