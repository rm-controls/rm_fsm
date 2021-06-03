//
// Created by peter on 2020/12/3.
//

#ifndef RM_FSM_COMMON_FSM_COMMON_H_
#define RM_FSM_COMMON_FSM_COMMON_H_

#include "rm_fsm/common/data.h"
#include "rm_fsm/common/command_sender.h"
#include "rm_fsm/common/controller_manager.h"

#include <iostream>
#include <queue>
#include <utility>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>

namespace rm_fsm {
enum MoveStatus { APPROACH_START, LEAVE_START, APPROACH_END, LEAVE_END };
class State {
 public:
  State(ros::NodeHandle &nh, Data *fsm_data, std::string state_name);
  virtual void run() {
    setChassis();
    setGimbal();
    setShooter();
    sendCommand(ros::Time::now());
  };
  void onEnter() { ROS_INFO("Enter %s state", state_name_.c_str()); }
  void onExit() { ROS_INFO("Exit %s state", state_name_.c_str()); }
  void updatePosStatus(MoveStatus move_status) { move_status_ = move_status; }
  std::string getName() { return state_name_; }
 protected:
  virtual void setChassis() { chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE); }
  virtual void setGimbal() { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::PASSIVE); }
  virtual void setShooter() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PASSIVE); }
  void sendCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  };
  ros::NodeHandle nh_;
  Data *data_;
  std::string state_name_;
  ChassisCommandSender *chassis_cmd_sender_;
  Vel2DCommandSender *vel_2d_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_;
  ShooterCommandSender *shooter_cmd_sender_;
  MoveStatus move_status_;
};

class Fsm {
 public:
  explicit Fsm(ros::NodeHandle &nh);
  ~Fsm() { delete controller_manager_; }
  enum { NORMAL, TRANSITIONING };
  virtual void run();
 protected:
  virtual std::string getDesiredState() = 0;
  void checkSwitch(const ros::Time &time);
  void remoteControlTurnOff() { controller_manager_->stopMovementControllers(); }
  void remoteControlTurnOn() { controller_manager_->startMovementControllers(); }

  ros::NodeHandle nh_;
  Data data_;
  ControllerManager *controller_manager_;
  State *current_state_, *next_state_;
  std::map<std::string, State *> string2state;
  std::string next_state_name_;
  int operating_mode_ = NORMAL;
  bool remote_is_open_{};
};
}
#endif // RM_FSM_COMMON_FSM_COMMON_H_
