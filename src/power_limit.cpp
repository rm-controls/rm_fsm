//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");
  power_nh.param("safety_effort", safety_effort_, 7.5 * 0.3);
  power_nh.param("ff", ff_, 0.3);
  power_nh.param("wheel_radius", wheel_radius_, 0.07625);
  power_nh.param("max_limit_50w", max_limit_50w_, 4.0);
  power_nh.param("max_limit_60w", max_limit_60w_, 4.0);
  power_nh.param("max_limit_70w", max_limit_70w_, 4.0);
  power_nh.param("max_limit_80w", max_limit_80w_, 4.0);
  power_nh.param("max_limit_100w", max_limit_100w_, 4.0);
  power_nh.param("max_limit_120w", max_limit_120w_, 4.0);

  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
  ros::NodeHandle pid_power_manager_nh = ros::NodeHandle(nh, "power_limit/pid_buffer_power_manager");

  if (!pid_buffer_.init(pid_nh)) ROS_INFO("[PowerLimit] PID initialize fail!");
  if (!pid_buffer_power_manager_.init(pid_power_manager_nh)) ROS_INFO("[PowerLimit] Power Manager PID initialize fail!");

  power_limit_pub_ = power_nh.advertise<rm_msgs::PowerLimit>("/limit_power", 1);
  joint_state_sub_ = power_nh.subscribe("/joint_states", 1, &PowerLimit::jointVelCB, this);

  lp_error_ = new LowPassFilter(20.0);
  ramp_error_ = new RampFilter<double>(400,0.01);
  pid_counter_ = 0.0;
}

void PowerLimit::input(RefereeData referee_data_,
                       PowerManagerData power_manager_data_,
                       bool use_power_manager) {

  if (use_power_manager) {
    ROS_INFO("USE POWER MANAGER");
    //init some data from power manager
    real_chassis_power_ = power_manager_data_.parameters[0];
    limit_power_ = power_manager_data_.parameters[1];
    capacity_ = power_manager_data_.parameters[3];
    error_power_ = limit_power_ - real_chassis_power_;
    have_capacity_ = true;

    ros::Time now = ros::Time::now();
    this->pid_buffer_power_manager_.computeCommand(error_power_, now - last_run_);
    last_run_ = now;

  } else {
    real_chassis_power_ = referee_data_.power_heat_data_.chassis_power;
    //limit_power_ = 50 + 70 * (sin(M_PI / 2 * last_run_.toSec()) > 0); //for test
    //limit_power_ = getLimitPower(referee_data_);
    limit_power_ = 100;

    if (referee_data_.power_heat_data_.chassis_power_buffer <= 30)
      limit_power_ -= 5.0;
    error_power_ = limit_power_ - real_chassis_power_;
    if (vel_total < 60.0)
      vel_total = 60.0;
    lp_error_->input(vel_total);
    vel_total = lp_error_->output();
    error_power_ = ((error_power_ / vel_total) / wheel_radius_ - ff_) / wheel_radius_;

    ramp_error_->input(error_power_);
    error_power_ = ramp_error_->output();

    have_capacity_ = false;

    if (pid_counter_ < 6) {
      pid_counter_++;
    }
    if (pid_counter_ == 6) {
      pid_counter_ = 0;
      ros::Time now = ros::Time::now();
      this->pid_buffer_.computeCommand(error_power_, now - last_run_);
      last_run_ = now;
    }

    power_limit_pub_data_.vel_total = vel_total;
    power_limit_pub_data_.limit_power = limit_power_;
    power_limit_pub_data_.effort = abs(pid_buffer_.getCurrentCmd());
    power_limit_pub_.publish(power_limit_pub_data_);
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

void PowerLimit::getLimitEffort() {
  if (this->limit_power_ <= 50) this->max_limit_ = max_limit_50w_;
  else if (this->limit_power_ <= 60 && this->limit_power_ > 50) this->max_limit_ = max_limit_60w_;
  else if (this->limit_power_ <= 70 && this->limit_power_ > 60) this->max_limit_ = max_limit_70w_;
  else if (this->limit_power_ <= 80 && this->limit_power_ > 70) this->max_limit_ = max_limit_80w_;
  else if (this->limit_power_ <= 100 && this->limit_power_ > 80) this->max_limit_ = max_limit_100w_;
  else if (this->limit_power_ <= 120 && this->limit_power_ > 100) this->max_limit_ = max_limit_120w_;

}

double PowerLimit::output() {
  /*
  this->getLimitEffort();

  if (have_capacity_ && ((abs(capacity_ - 0.25) <= 0.05) || capacity_ < 0.25))
    return safety_effort_;
  else {
    if (have_capacity_) {
      if (abs(pid_buffer_power_manager_.getCurrentCmd()) > max_limit_) return max_limit_;
      else return abs(pid_buffer_power_manager_.getCurrentCmd());
    } else {
      if (abs(pid_buffer_.getCurrentCmd()) > max_limit_) return max_limit_;
      else return abs(pid_buffer_.getCurrentCmd());
    }
  }
   */
  if (have_capacity_ && ((abs(capacity_ - 0.25) <= 0.05) || capacity_ < 0.25))
    return safety_effort_;
  else {
    if (have_capacity_) {
      return abs(pid_buffer_power_manager_.getCurrentCmd());
    } else {
      return abs(pid_buffer_.getCurrentCmd());
    }
  }
}

