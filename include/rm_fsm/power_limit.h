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
             bool use_power_manager, bool k_shift);
  double output() const;
  double getLimitPower(RefereeData referee_data_);
  double getSafetyEffort(bool enter_over_power_mode);

 private:
  double effort_ = 99;
  double coeff_{};
  double multiple_{};
  double danger_surplus_{};
  double roll_back_buffer_{};
  double capacity_surplus_{};
  double safety_current_{};

  bool end_over_power_mode_flag_;

};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
