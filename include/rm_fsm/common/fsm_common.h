//
// Created by peter on 2020/12/3.
//

#ifndef RM_FSM_COMMON_FSM_COMMON_H_
#define RM_FSM_COMMON_FSM_COMMON_H_

#include <iostream>
#include <queue>
#include <utility>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include "rm_fsm/common/data.h"
#include "rm_fsm/common/command_sender.h"

namespace rm_fsm {
class State {
 public:
  State(ros::NodeHandle &nh, Data *fsm_data, std::string state_name);
  void sendCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  };
  virtual void onEnter() = 0;
  virtual void run() = 0;
  virtual void onExit() = 0;

  ros::NodeHandle nh_;
  Data *data_;
  std::string state_name_;     // enumerated name of the current state

  ChassisCommandSender *chassis_cmd_sender_;
  VelCommandSender *vel_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_;
  ShooterCommandSender *shooter_cmd_sender_;
};

class Fsm {
 public:
  explicit Fsm(ros::NodeHandle &nh);
  enum { NORMAL, TRANSITIONING };
  void run();
  virtual std::string getDesiredState() = 0;

  ros::NodeHandle nh_;
  Data data_;
 protected:
  State *current_state_;    // current FSM state
  State *next_state_;       // next FSM state
  std::map<std::string, State *> string2state;
  std::string next_state_name_;  // next FSM state name
  int operating_mode_ = NORMAL;
};
}
#endif // RM_FSM_COMMON_FSM_COMMON_H_
