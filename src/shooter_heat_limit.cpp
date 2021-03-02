//
// Created by bruce on 2021/2/10.
//

#include "rm_fsm/shooter_heat_limit.h"

ShooterHeatLimit::ShooterHeatLimit(ros::NodeHandle &nh) {
  ros::NodeHandle shooter_heat_limit_nh = ros::NodeHandle(nh, "shooter_heat_limit");
  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
}

void ShooterHeatLimit::input(referee::RefereeData referee) {
  uint16_t shooter_heat_max = referee.game_robot_status_.shooter_heat0_cooling_limit;
  uint16_t shooter_cooling_rate = referee.game_robot_status_.shooter_heat0_cooling_rate;
  uint16_t shooter_heat = referee.power_heat_data_.shooter_heat0;
  uint8_t bullet_heat = 10;
  bool shoot_result = false;//judge if robot shoots a bullet, its value need  to be changed by msgs.

  if (shooter_heat_max <= 600 && shooter_heat_max != 0) {
    //plan A
    if (shoot_result) {
      ShooterHeatLimit::hz =
          static_cast<uint8_t>(10 * ((shooter_heat_max - shooter_heat) - shooter_cooling_rate) / bullet_heat);
    } else if (1) {
      if (shooter_heat >= shooter_heat_max - 5) {
        hz = 0;
      } else {
        ShooterHeatLimit::hz =
            static_cast<uint8_t>((shooter_heat_max - shooter_heat + shooter_cooling_rate - 5) / bullet_heat);
      }
    }
  } else {
    hz = 10;
  }

}

uint8_t ShooterHeatLimit::output() {
  return hz;
}
