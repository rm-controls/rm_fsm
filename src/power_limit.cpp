//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");
  power_nh.param("safety_effort", safety_effort_, 7.5 * 0.3);
  power_nh.param("coeff", coeff_, 0.3);

  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
  if (!pid_buffer_.init(pid_nh))
    ROS_INFO("[PowerLimit] PID initialize fail!");

  ramp_chassis_power = new RampFilter<double>(10, 0.001);
  limit_power_pub_ = power_nh.advertise<rm_msgs::Referee>("/limit_power", 1);
  joint_state_sub_ = power_nh.subscribe("/joint_state", 1, &PowerLimit::jointVelCB, this);
  pid_counter_ = 0.0;
}

void PowerLimit::input(RefereeData referee_data_,
                       PowerManagerData power_manager_data_,
                       bool use_power_manager) {

  if (use_power_manager) {
    //init some data from power manager
    real_chassis_power_ = power_manager_data_.parameters[0];
    limit_power_ = power_manager_data_.parameters[1];
    capacity_ = power_manager_data_.parameters[3];
    error_power_ = real_chassis_power_ - limit_power_;
    have_capacity_ = true;

    ramp_chassis_power->input(real_chassis_power_);
    real_chassis_power_ = ramp_chassis_power->output();
    ramp_chassis_power->clear(power_manager_data_.parameters[0]);

  } else {
    real_chassis_power_ = referee_data_.power_heat_data_.chassis_power;
    limit_power_ = 20 + 40 * (sin(2 * M_PI * last_run_.toSec()) > 0); //for test
    //limit_power_ = 60;


    error_power_ = limit_power_ - real_chassis_power_;
    if (vel_total < 0.5) vel_total = 1.0;
    error_power_ = coeff_ * error_power_ / vel_total;
    have_capacity_ = false;

    limit_power_pub_data_.chassis_power = error_power_;
    limit_power_pub_data_.chassis_current = limit_power_;
    limit_power_pub_.publish(limit_power_pub_data_);
/*
    ramp_chassis_power->input(real_chassis_power_);
    real_chassis_power_ = ramp_chassis_power->output();
    ramp_chassis_power->clear(referee_data_.power_heat_data_.chassis_power);
*/
  }

  if (pid_counter_ < 20) {
    pid_counter_++;
  }
  if (pid_counter_ == 20) {
    pid_counter_ = 0;
    ros::Time now = ros::Time::now();
    this->pid_buffer_.computeCommand(error_power_, now - last_run_);
    last_run_ = now;
  }

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

void PowerLimit::jointVelCB(const sensor_msgs::JointState &data) {
  vel_total = abs(data.velocity[0]) + abs(data.velocity[1]) + abs(data.velocity[2]) + abs(data.velocity[3]);
}

double PowerLimit::output() {
  if (have_capacity_ && ((abs(capacity_ - 0.25) <= 0.05) || capacity_ < 0.25))
    return safety_effort_;
  else {
    //ROS_INFO("%f", abs(pid_buffer_.getCurrentCmd()));
    return abs(pid_buffer_.getCurrentCmd());
  }
}

