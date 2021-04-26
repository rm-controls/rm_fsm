//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");
  power_nh.param("safety_effort", safety_effort_, 7.5 * 0.3);
  power_nh.param("ff", ff_, 0.3);
  power_nh.param("wheel_radius", wheel_radius_, 0.07625);

  //dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_fsm::PowerLimitConfig>(nh);
  dynamic_reconfigure::Server<rm_fsm::PowerLimitConfig>::CallbackType cb =
      [this](auto &&PH1, auto &&PH2) {
        reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
      };
  d_srv_->setCallback(cb);

  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
  ros::NodeHandle pid_power_manager_nh = ros::NodeHandle(nh, "power_limit/pid_buffer_power_manager");

  if (!pid_buffer_.init(pid_nh)) ROS_INFO("[PowerLimit] PID initialize fail!");
  if (!pid_buffer_power_manager_.init(pid_power_manager_nh)) ROS_INFO("[PowerLimit] Power Manager PID initialize fail!");

  power_limit_pub_ = power_nh.advertise<rm_msgs::PowerLimit>("/Powerlimit", 1);
  joint_state_sub_ = power_nh.subscribe("/joint_states", 1, &PowerLimit::jointVelCB, this);

  lp_vel_total_ = new LowPassFilter(100.0);
  lp_real_power_ = new LowPassFilter(40.0);
  ramp_effort_ = new RampFilter<double>(200, 0.01);
  pid_counter_ = 0.0;
}

void PowerLimit::input(RefereeData referee_data_,
                       PowerManagerData power_manager_data_,
                       bool use_power_manager) {
  double tmp_vel_total_ = 0.0;
  if (use_power_manager) {
    //init some data from power manager
    real_chassis_power_ = power_manager_data_.parameters[0];
    limit_power_ = power_manager_data_.parameters[1];
    capacity_ = power_manager_data_.parameters[3];
    //filter the real power
    lp_real_power_->input(real_chassis_power_);
    real_chassis_power_ = lp_real_power_->output();

    error_power_ = limit_power_ - real_chassis_power_;
    have_capacity_ = true;

    tmp_vel_total_ = vel_total;
    if (tmp_vel_total_ < 60.0) //give a const value when the total velocity is too low
      tmp_vel_total_ = 60.0 + 5.0 * (last_vel_total_ - vel_total);

    error_effort_ = ((error_power_ / tmp_vel_total_) / wheel_radius_ + ff_) / wheel_radius_;//need to be verified
    //filer the error effort
    ramp_effort_->input(error_effort_);
    error_power_ = ramp_effort_->output();

    ros::Time now = ros::Time::now();
    this->pid_buffer_power_manager_.computeCommand(error_effort_, now - last_run_);
    last_run_ = now;

  } else {
    //init some data from referee
    real_chassis_power_ = referee_data_.power_heat_data_.chassis_power;
    limit_power_ = referee_data_.game_robot_status_.chassis_power_limit;
    tmp_vel_total_ = vel_total;
    error_power_ = limit_power_ - real_chassis_power_;
    if (tmp_vel_total_ < 60.0)
      tmp_vel_total_ = 60.0;
    error_effort_ = ((error_power_ / tmp_vel_total_) / wheel_radius_ + ff_) / wheel_radius_;

    ramp_effort_->input(error_power_);
    error_power_ = ramp_effort_->output();

    have_capacity_ = false;

    if (pid_counter_ < 6) {
      pid_counter_++;
    }
    if (pid_counter_ == 6) {
      pid_counter_ = 0;
      ros::Time now = ros::Time::now();
      this->pid_buffer_.computeCommand(error_effort_, now - last_run_);
      last_run_ = now;
    }
  }
  //pubilsh the data for test
  power_limit_pub_data_.vel_total = tmp_vel_total_;
  power_limit_pub_data_.limit_power = limit_power_;
  power_limit_pub_data_.effort = abs(pid_buffer_.getCurrentCmd());
  power_limit_pub_data_.real_power = real_chassis_power_;
  power_limit_pub_.publish(power_limit_pub_data_);
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

void PowerLimit::reconfigCB(rm_fsm::PowerLimitConfig &config, uint32_t) {
  ROS_INFO("[Fsm] Dynamic params change");
  limit_power_ = config.limit_power;
  power_offset_ = config.power_offset;
}

void PowerLimit::jointVelCB(const sensor_msgs::JointState &data) {
  last_vel_total_ = vel_total;
  vel_total = abs(data.velocity[0]) + abs(data.velocity[1]) + abs(data.velocity[2]) + abs(data.velocity[3]);
  lp_vel_total_->input(vel_total);
  vel_total = lp_vel_total_->output();
}

double PowerLimit::output() {
  if (have_capacity_ && ((abs(capacity_ - 0.25) <= 0.05) || capacity_ < 0.25))
    return safety_effort_;
  else {
    if (have_capacity_) {
      return abs(pid_buffer_power_manager_.getCurrentCmd());
    } else {
      return abs(pid_buffer_.getCurrentCmd()) + power_offset_;
    }
  }
}

