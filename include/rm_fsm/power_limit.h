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
  ros::Time last_run_;
  control_toolbox::Pid pid_buffer_;
  double des_buffer_ = 0.0;

};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
