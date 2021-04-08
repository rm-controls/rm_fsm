//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");

  power_nh.param("danger_surplus", danger_surplus_, 10.0);
  power_nh.param("roll_back_buffer", roll_back_buffer_, 10.0);
  power_nh.param("coeff", coeff_, 0.1);
  power_nh.param("multiple", multiple_, 5.0);
  power_nh.param("capacity_surplus", capacity_surplus_, 0.23);
  power_nh.param("safety_current", safety_current_, 2.0);

  end_over_power_mode_flag_ = false;

}

void PowerLimit::input(RefereeData referee_data_,
                       PowerManagerData power_manager_data_,
                       bool use_power_manager, bool k_shift) {
  uint16_t w0 = referee_data_.power_heat_data_.chassis_power_buffer;
  double w1, w2;              //chassis power buffer 100ms&200ms later
  double chassis_power;
  double chassis_voltage;
  double chassis_capacity;
  double chassis_current_limit;
  double limit_power;

  if (use_power_manager) {
    if (k_shift) {
      //ROS_INFO_THROTTLE(10, "Enter normal mode,with power manage!");
      chassis_power = power_manager_data_.parameters[0];        //real power
      limit_power = power_manager_data_.parameters[1];          //limit power
      chassis_voltage = power_manager_data_.parameters[2];      //power
      chassis_capacity = power_manager_data_.parameters[3];     //capacity
      w0 = 60 * chassis_capacity;

      chassis_current_limit = limit_power / chassis_voltage;

      if (chassis_power <= limit_power && !end_over_power_mode_flag_) {
        this->effort_ = 99;
      } else {

        if (chassis_capacity <= capacity_surplus_ || end_over_power_mode_flag_) {
          this->effort_ = (chassis_current_limit + 5 * w0 / chassis_voltage) * coeff_;
          end_over_power_mode_flag_ = true;
        } else {
          this->effort_ = 99;
        }
      }
      //if (end_over_power_mode_flag_ && chassis_capacity > 0.22) this->current_ = chassis_current_limit * coeff;
      if (chassis_capacity > 0.9) this->end_over_power_mode_flag_ = false;
    } else {
      this->effort_ = getSafetyEffort(end_over_power_mode_flag_);
    }

  } else {
    if (k_shift) {
      //ROS_INFO_THROTTLE(10, "Enter normal mode,without manage!");
      chassis_voltage = referee_data_.power_heat_data_.chassis_volt;
      chassis_power = referee_data_.power_heat_data_.chassis_power;
      limit_power = getLimitPower(referee_data_);

      chassis_current_limit = limit_power / chassis_voltage;

      if (chassis_power <= limit_power && w0 >= roll_back_buffer_) {
        //ROS_INFO_THROTTLE(1, "Didn't use buffer power.");
        this->effort_ = 99;
      } else {
        w1 = w0 - 0.1 * (chassis_power - limit_power);
        w2 = w1 - 0.1 * (chassis_power - limit_power);
        if (w2 < danger_surplus_) {
          // ROS_INFO_THROTTLE(1, "After 200ms later,buffer power less than 10J,begin to limit.");
          // GUET plan
          this->effort_ = (chassis_current_limit + 5 * w0 / chassis_voltage) * coeff_;
        } else {
          this->effort_ = (chassis_current_limit + multiple_ * w0 / chassis_voltage) * coeff_;
          //  ROS_INFO_THROTTLE(1, "After 200ms later,buffer power more than 10J,safe.");
        }
      }
    } else {
      this->effort_ = getSafetyEffort(false);
    }
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

double PowerLimit::getSafetyEffort(bool enter_over_power_mode) {
  if (enter_over_power_mode) {
    return 0.8 * coeff_;
  } else {
    return safety_current_ * coeff_;
  }
}

double PowerLimit::output() const {
  return effort_;
}
