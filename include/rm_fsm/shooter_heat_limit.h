//
// Created by bruce on 2021/2/10.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_SHOOTER_HEAT_LIMIT_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_SHOOTER_HEAT_LIMIT_H_

#include "ros/ros.h"
#include "referee.h"

class ShooterHeatLimit {
 public:
  explicit ShooterHeatLimit(ros::NodeHandle &nh);
  void input(referee::RefereeData referee);
  uint8_t output();

 private:
  uint8_t hz = 0;
};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_SHOOTER_HEAT_LIMIT_H_
