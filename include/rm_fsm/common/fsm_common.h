//
// Created by peter on 2020/12/3.
//

#ifndef RM_FSM_COMMON_FSM_COMMON_H_
#define RM_FSM_COMMON_FSM_COMMON_H_

#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_loader.h>
#include <rm_common/decision/calibration_manager.h>
#include "rm_fsm/common/data.h"

namespace rm_fsm {
class StateBase {
 public:
  StateBase(ros::NodeHandle &nh, Data *data, std::string state_name)
      : nh_(nh), data_(data), state_name_(std::move(state_name)) {
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_->referee_);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_->referee_);
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_->referee_);
  }
  virtual void run() {
    setChassis();
    setGimbal();
    setShooter();
    sendCommand(ros::Time::now());
  }
  void onEnter() { ROS_INFO("Enter %s state", state_name_.c_str()); }
  void onExit() { ROS_INFO("Exit %s state", state_name_.c_str()); }
  std::string getName() { return state_name_; }
 protected:
  virtual void setChassis() { chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW); }
  virtual void setGimbal() { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  virtual void setShooter() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void sendCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  };
  ros::NodeHandle nh_;
  Data *data_;
  std::string state_name_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  rm_common::GimbalCommandSender *gimbal_cmd_sender_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_;
};

class FsmBase {
 public:
  explicit FsmBase(ros::NodeHandle &nh);
  ~FsmBase() {
    delete controller_loader_;
    delete calibration_manager_;
  }
  enum { NORMAL, TRANSITIONING };
  virtual void run();
 protected:
  virtual std::string getNextState() = 0;
  void checkSwitch(const ros::Time &time);
  void checkReferee(const ros::Time &time);

  // Referee
  virtual void chassisOutputOn() {};
  virtual void gimbalOutputOn() {};
  virtual void shooterOutputOn() {};

  // Remote Controller
  virtual void remoteControlTurnOff() {
    switch_base_ctrl_srv_->flipControllers();
    switch_base_ctrl_srv_->callService();
  }
  virtual void remoteControlTurnOn() {
    switch_base_ctrl_srv_->switchControllers();
    switch_base_ctrl_srv_->callService();
    calibration_manager_->reset();
  }

  ros::NodeHandle nh_;
  Data data_;
  rm_common::ControllerLoader *controller_loader_;
  rm_common::CalibrationManager *calibration_manager_;
  rm_common::SwitchControllersService *switch_state_ctrl_srv_, *switch_base_ctrl_srv_{};

  StateBase *current_state_, *next_state_;
  std::map<std::string, StateBase *> string2state;
  std::string next_state_name_;
  int operating_mode_ = NORMAL;
  bool remote_is_open_{};
};
}
#endif // RM_FSM_COMMON_FSM_COMMON_H_
