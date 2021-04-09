//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");
  power_nh.param("safety_effort", safety_effort_, 7.5 * 0.3);

  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
  if (!pid_buffer_.init(pid_nh))
    ROS_INFO("[PowerLimit] PID initialize fail!");

}

void PowerLimit::input(RefereeData referee_data_,
                       PowerManagerData power_manager_data_,
                       bool use_power_manager) {

  if (use_power_manager) {
    //init some data from power manager
    real_chassis_power_ = power_manager_data_.parameters[0];
    limit_power_ = power_manager_data_.parameters[1];
    error_power_ = real_chassis_power_ - limit_power_;

  } else {
    real_chassis_power_ = referee_data_.power_heat_data_.chassis_power;
    limit_power_ = getLimitPower(referee_data_);
    error_power_ = real_chassis_power_ - limit_power_;
  }

  ros::Time now = ros::Time::now();
  this->pid_buffer_.computeCommand(error_power_, now - last_run_);
  last_run_ = now;
}

// Change limit power of different level and robot
double PowerLimit::getLimitPower(RefereeData referee_data_) {
  double limit_power;
  if ((referee_data_.game_robot_status_.robot_id >= 3
      && referee_data_.game_robot_status_.robot_id <= 5)
      || (referee_data_.game_robot_status_.robot_id >= 103
          && referee_data_.game_robot_status_.robot_id <= 105)) { // Standard robot
    if (referee_data_.performance_system_ == 0) // Power first
      limit_power = 50 + 10 * referee_data_.game_robot_status_.robot_level;
    else // Hp first
      limit_power = 50 + 5 * referee_data_.game_robot_status_.robot_level;
  } else if (referee_data_.game_robot_status_.robot_id == 1
      || referee_data_.game_robot_status_.robot_id == 101) { // Hero robot
    if (referee_data_.performance_system_ == 0) // Power first
      if (referee_data_.game_robot_status_.robot_level == 3)
        limit_power = 120;
      else
        limit_power = 50 + 20 * referee_data_.game_robot_status_.robot_level;
    else { // Hp first
      limit_power = 50 + 5 * referee_data_.game_robot_status_.robot_level;
    }
  } else if (referee_data_.game_robot_status_.robot_id == 7
      || referee_data_.game_robot_status_.robot_id == 107) { // Sentry robot
    limit_power = 20;
  } else { // Other robots
    limit_power = 200;
  }
  return limit_power;
}

double PowerLimit::getSafetyEffort() {

  return safety_effort_;
}

double PowerLimit::output() {
  return abs(pid_buffer_.getCurrentCmd());
}

