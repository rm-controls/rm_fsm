//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  if (!pid_buffer_.init(nh))
    ROS_INFO("[PowerLimit] PID initialize fail!");

}

void PowerLimit::input(referee::RefereeData referee, const ros::Duration &period) {
  double real_buffer = referee.power_heat_data_.chassis_power_buffer;
  double des_buffer = 0.0;
  double error_buffer = real_buffer - des_buffer;

  this->pid_buffer_.computeCommand(error_buffer, period);

}

double PowerLimit::output() {
  return pid_buffer_.getCurrentCmd();
}
