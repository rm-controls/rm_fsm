//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");
  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
  if (!pid_buffer_.init(pid_nh))
    ROS_INFO("[PowerLimit] PID initialize fail!");
  power_nh.param("des_buffer", des_buffer_, 1.0);
}

void PowerLimit::input(referee::RefereeData referee) {
  double real_buffer = referee.power_heat_data_.chassis_power_buffer;
  double error_buffer = real_buffer - des_buffer_;
  ros::Time now = ros::Time::now();
  this->pid_buffer_.computeCommand(error_buffer, now - last_run_);
  last_run_ = now;
}

double PowerLimit::output() {
  return abs(pid_buffer_.getCurrentCmd());
}
