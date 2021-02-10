//
// Created by bruce on 2021/2/10.
//

#include "rm_fsm/shooter_heat_limit.h"

ShooterHeatLimit::ShooterHeatLimit(ros::NodeHandle &nh) {
  ros::NodeHandle shooter_heat_limit_nh = ros::NodeHandle(nh, "shooter_heat_limit");
  ros::NodeHandle pid_nh = ros::NodeHandle(nh, "power_limit/pid_buffer");
}

void ShooterHeatLimit::input(referee::RefereeData referee) {
  uint16_t shooter_haet_max = referee.game_robot_status_.shooter_heat0_cooling_limit;
  uint16_t shooter_cooling_rate = referee.game_robot_status_.shooter_heat0_cooling_rate;
  uint16_t shooter_heat = referee.power_heat_data_.shooter_heat0;
  uint16_t shooter_cooling_buff = 1;
  uint16_t fighting_time = 3;
  uint8_t bullet_heat = 10;
  bool shoot_result = true;//judge if robot shoots a bullet, its value need be changed by msgs.


  //judge what buff does robot gets
  if (referee.buff_.power_rune_buff & 0x00000010) {//judge if the robot gets buff or not
    switch (referee.rfid_status_.rfid_status) {//judge what buff does robot gets
      case 0x00000001: shooter_cooling_buff = 3;
      case 0x00000010: shooter_cooling_buff = 5;
      case 0x00000100: shooter_cooling_buff = 5;
      case 0x00001000: shooter_cooling_buff = 3;
    }
  }
  //plan A
  if (shoot_result) {
    ShooterHeatLimit::hz =
        static_cast<uint8_t>(10 * ((shooter_haet_max - shooter_heat) - shooter_cooling_rate) / bullet_heat);
  }

  //plan B
  if (shooter_heat == 0) {
    ShooterHeatLimit::hz =
        static_cast<uint8_t>((shooter_haet_max / fighting_time + shooter_cooling_rate * shooter_cooling_buff)
            / bullet_heat);
  } else {
    ShooterHeatLimit::hz =
        static_cast<uint8_t>(shooter_cooling_rate * shooter_cooling_buff / bullet_heat);
  }
}

uint8_t ShooterHeatLimit::output() {
  return hz;
}