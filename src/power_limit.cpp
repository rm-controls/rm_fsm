//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit/");
  if (!pid_buffer_.init(power_nh))
    ROS_INFO("[PowerLimit] PID initialize fail!");
  nh.param("des_buffer_", des_buffer_, 0.0);
}

void PowerLimit::input(referee::RefereeData referee) {
  double real_buffer = referee.power_heat_data_.chassis_power_buffer;
  double error_buffer = real_buffer - des_buffer_;
  ros::Time now = ros::Time::now();
  this->pid_buffer_.computeCommand(error_buffer, now - last_run_);
  last_run_ = now;
}

double PowerLimit::output() {
  return pid_buffer_.getCurrentCmd();
}
