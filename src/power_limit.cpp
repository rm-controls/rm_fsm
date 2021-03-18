//
// Created by kiana on 2021/1/26.
//

#include "rm_fsm/power_limit.h"

PowerLimit::PowerLimit(ros::NodeHandle &nh) {
  ros::NodeHandle power_nh = ros::NodeHandle(nh, "power_limit");

  power_nh.param("danger_surplus_", danger_surplus_, 10.0);
  power_nh.param("roll_back_buffer_", roll_back_buffer_, 10.0);
  power_nh.param("coeff", coeff, 0.1);
  power_nh.param("multiple", multiple, 5.0);

}

void PowerLimit::input(referee::RefereeData referee) {
  double chassis_power = referee.power_heat_data_.chassis_power;
  double chassis_voltage = referee.power_heat_data_.chassis_volt;
  double chassis_current_need = 99;
  double chassis_current_limit;
  uint16_t w0 = referee.power_heat_data_.chassis_power_buffer; //chassis power buffer now
  double w1, w2;              //chassis power buffer 100ms&200ms later
  double limit_power;

  // Change limit power of different level and robot
  if ((referee.game_robot_status_.robot_id >= 3
      && referee.game_robot_status_.robot_id <= 5)
      || (referee.game_robot_status_.robot_id >= 103
          && referee.game_robot_status_.robot_id <= 105)) { // Standard robot
    if (referee.performance_system_ == 0) // Power first
      limit_power = 40 + 10 * referee.game_robot_status_.robot_level;
    else // Hp first
      limit_power = 40 + 5 * referee.game_robot_status_.robot_level;
  } else if (referee.game_robot_status_.robot_id == 1
      || referee.game_robot_status_.robot_id == 101) { // Hero robot
    if (referee.performance_system_ == 0) // Power first
      if (referee.game_robot_status_.robot_level == 3)
        limit_power = 120;
      else
        limit_power = 50 + 20 * referee.game_robot_status_.robot_level;
    else { // Hp first
      limit_power = 50 + 5 * referee.game_robot_status_.robot_level;
    }
  } else if (referee.game_robot_status_.robot_id == 7
      || referee.game_robot_status_.robot_id == 107) { // Sentry robot
    limit_power = 20;
  } else { // Other robots
    limit_power = 200;
  }

  chassis_current_limit = limit_power / chassis_voltage;
  //this->openCapacity();        //check the super capacity button open or not
  //limit_power = limit_power + this->getCapacity();


  if (chassis_power <= limit_power && w0 >= roll_back_buffer_) {
    ROS_INFO_THROTTLE(1, "Didn't use buffer power.");
    chassis_current_need = 99;
    this->current_ = chassis_current_need;
  } else {
    w1 = w0 - 0.1 * (chassis_power - limit_power);
    w2 = w1 - 0.1 * (chassis_power - limit_power);
    if (w2 < danger_surplus_) {
      ROS_INFO_THROTTLE(1, "After 200ms later,buffer power less than 10J,begin to limit.");
      // GUET plan
      chassis_current_need = (chassis_current_limit + 5 * w0 / chassis_voltage);
      this->current_ = chassis_current_need * coeff;
    } else {
      this->current_ = (chassis_current_limit + multiple * w0 / chassis_voltage) * coeff;
      ROS_INFO_THROTTLE(1, "After 200ms later,buffer power more than 10J,safe.");
    }
  }
}

double PowerLimit::output() const {
  return current_;
}
