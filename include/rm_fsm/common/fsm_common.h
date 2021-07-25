//
// Created by peter on 2020/12/3.
//

#ifndef RM_FSM_COMMON_FSM_COMMON_H_
#define RM_FSM_COMMON_FSM_COMMON_H_

#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>

#include "rm_fsm/common/data.h"
#include "rm_common/decision/controller_manager.h"
#include "rm_common/decision/calibration_queue.h"

namespace rm_fsm {
class StateBase {
 public:
  StateBase(ros::NodeHandle &nh, Data *data, std::string state_name)
      : nh_(nh), data_(data), state_name_(std::move(state_name)) {
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_->referee_.referee_data_);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    ros::NodeHandle upper_gimbal_nh(nh, "upper_gimbal");
    upper_gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(upper_gimbal_nh, data_->referee_.referee_data_);
    ros::NodeHandle lower_gimbal_nh(nh, "lower_gimbal");
    lower_gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(upper_gimbal_nh, data_->referee_.referee_data_);
    ros::NodeHandle upper_shooter_nh(nh, "upper_shooter");
    upper_shooter_cmd_sender_ = new rm_common::ShooterCommandSender(upper_shooter_nh, data_->referee_.referee_data_);
    ros::NodeHandle lower_shooter_nh(nh, "lower_shooter");
    upper_shooter_cmd_sender_ = new rm_common::ShooterCommandSender(lower_shooter_nh, data_->referee_.referee_data_);
  }
  virtual void run() {
    setChassis();
    setUpperGimbal();
    setLowerGimbal();
    setUpperShooter();
    setLowerShooter();
    sendCommand(ros::Time::now());
  }
  void onEnter() { ROS_INFO("Enter %s", state_name_.c_str()); }
  std::string getName() { return state_name_; }
 protected:
  virtual void setChassis() {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  }
  virtual void setUpperGimbal() { upper_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  virtual void setLowerGimbal() { lower_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  virtual void setUpperShooter() { upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  virtual void setLowerShooter() { lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void sendCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->sendCommand(time);
    upper_gimbal_cmd_sender_->sendCommand(time);
    lower_gimbal_cmd_sender_->sendCommand(time);
    upper_shooter_cmd_sender_->sendCommand(time);
    lower_shooter_cmd_sender_->sendCommand(time);
  };
  ros::NodeHandle nh_;
  Data *data_;
  std::string state_name_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  rm_common::GimbalCommandSender *upper_gimbal_cmd_sender_;
  rm_common::GimbalCommandSender *lower_gimbal_cmd_sender_;
  rm_common::ShooterCommandSender *upper_shooter_cmd_sender_;
  rm_common::ShooterCommandSender *lower_shooter_cmd_sender_;
};

class FsmBase {
 public:
  explicit FsmBase(ros::NodeHandle &nh);
  ~FsmBase() {

  }
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
    controller_manager_.stopMainControllers();
    controller_manager_.stopCalibrationControllers();
  }
  virtual void remoteControlTurnOn() {
    controller_manager_.startMainControllers();
  }

  ros::NodeHandle nh_;
  Data data_;
  rm_common::ControllerManager controller_manager_;
  rm_common::ControllerManager calibration_loader;
  std::vector<std::string> main_controllers_, calibration_controllers_,state_controllers_;

  StateBase *current_state_;
  std::map<std::string, StateBase *>   string2state;
  bool remote_is_open_{};
  bool chassis_output_{}, gimbal_output_{}, shooter_output_{};
};
}
#endif // RM_FSM_COMMON_FSM_COMMON_H_
