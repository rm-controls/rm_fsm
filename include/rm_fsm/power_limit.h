//
// Created by kiana on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include "rm_fsm/referee.h"

class PowerLimit {
 public:
  explicit PowerLimit(ros::NodeHandle &nh);
  void input(RefereeData referee_data_,
             PowerManagerData power_manager_data_,
             bool use_power_manager);
  double output();
  double getLimitPower(RefereeData referee_data_);
  double getSafetyEffort();

 private:
  ros::Time last_run_;
  control_toolbox::Pid pid_buffer_;

  double real_chassis_power_;
  double limit_power_;
  double capacity_;
  double error_power_;

  double safety_effort_;

  bool have_capacity_;

};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
