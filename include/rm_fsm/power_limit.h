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
  void input(RefereeData referee_data_,
             PowerManagerData power_manager_data_,
             bool use_power_manager,
             bool k_shift = false);
  double output() const;
  double getLimitPower(RefereeData referee_data_);

 private:
  double current_ = 99;
  double coeff{};
  double multiple{};
  double danger_surplus_{};
  double roll_back_buffer_{};
  double capacity_surplus_{};
};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
