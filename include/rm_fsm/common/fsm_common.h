//
// Created by peter on 2020/12/3.
//

#ifndef RM_FSM_COMMON_FSM_COMMON_H_
#define RM_FSM_COMMON_FSM_COMMON_H_

#include "rm_fsm/common/data.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>

namespace rm_fsm {
class StateBase {
 public:
  StateBase(ros::NodeHandle &nh, Data *data, std::string state_name);
  virtual void run();
  void onEnter() { ROS_INFO("Enter %s", state_name_.c_str()); }
  std::string getName() { return state_name_; }
 protected:
  void sendCommand(const ros::Time &time);
  virtual void setChassis() { chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW); }
  virtual void setUpperGimbal() { upper_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  virtual void setLowerGimbal() { lower_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  virtual void setUpperShooter() { upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  virtual void setLowerShooter() { lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }

  Data *data_;
  std::string state_name_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  rm_common::GimbalCommandSender *upper_gimbal_cmd_sender_, *lower_gimbal_cmd_sender_;
  rm_common::ShooterCommandSender *upper_shooter_cmd_sender_, *lower_shooter_cmd_sender_;
};

class FsmBase {
 public:
  explicit FsmBase(ros::NodeHandle &nh);
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
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();

  StateBase *current_state_;
  Data data_;
  ros::NodeHandle nh_;
  rm_common::ControllerManager controller_manager_;
  rm_common::CalibrationQueue *trigger_calibration_;
  std::map<std::string, StateBase *> string2state_;
  bool remote_is_open_{};
  bool chassis_output_{}, gimbal_output_{}, shooter_output_{};
};
}
#endif // RM_FSM_COMMON_FSM_COMMON_H_
