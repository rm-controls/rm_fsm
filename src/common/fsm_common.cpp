//
// Created by peter on 2020/12/3.
//

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
StateBase::StateBase(ros::NodeHandle &nh, Data *data, std::string state_name)
    : data_(data), state_name_(std::move(state_name)) {
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_->referee_.referee_data_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  ros::NodeHandle upper_nh(nh, "upper");
  upper_cmd_sender_ = new SideCommandSender(upper_nh, data_->referee_.referee_data_, data_->upper_track_data_array_,
                                            data_->upper_gimbal_des_error_, data_->upper_yaw_, data_->upper_pitch_);
  ros::NodeHandle lower_nh(nh, "lower");
  lower_cmd_sender_ = new SideCommandSender(lower_nh, data_->referee_.referee_data_, data_->lower_track_data_array_,
                                            data_->lower_gimbal_des_error_, data_->lower_yaw_, data_->lower_pitch_);
}

void StateBase::run() {
  setChassis();
  setGimbal(upper_cmd_sender_);
  setGimbal(lower_cmd_sender_);
  setShooter(upper_cmd_sender_);
  setShooter(lower_cmd_sender_);
  sendCommand(ros::Time::now());
}

void StateBase::sendCommand(const ros::Time &time) {
  chassis_cmd_sender_->sendCommand(time);
  vel_2d_cmd_sender_->sendCommand(time);
  upper_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  upper_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
}

FsmBase::FsmBase(ros::NodeHandle &nh) : data_(nh), nh_(nh), controller_manager_(nh) {
  ros::NodeHandle upper_nh(nh, "upper");
  ros::NodeHandle upper_detection_switch_nh(upper_nh, "detection_switch");
  upper_switch_detection_srv_ = new rm_common::SwitchDetectionCaller(upper_detection_switch_nh);
  ros::NodeHandle lower_nh(nh, "lower");
  ros::NodeHandle lower_detection_switch_nh(lower_nh, "detection_switch");
  lower_switch_detection_srv_ = new rm_common::SwitchDetectionCaller(lower_detection_switch_nh);
  controller_manager_.startStateControllers();
  string2state_.insert(std::make_pair("INVALID", nullptr));
  current_state_ = string2state_["INVALID"];
}

void FsmBase::run() {
  ros::Time time = ros::Time::now();
  data_.update(time);
  checkReferee(time);
  checkSwitch(time);
  controller_manager_.update();
  upper_switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
  upper_switch_detection_srv_->callService();
  lower_switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
  lower_switch_detection_srv_->callService();
  std::string next_state_name = getNextState();
  if (next_state_name != current_state_->getName()) {
    current_state_ = string2state_[next_state_name];
    current_state_->onEnter();
    current_state_->run();
  } else current_state_->run();
}

void FsmBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.1) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.1) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
}

void FsmBase::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
}

void FsmBase::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
}

void FsmBase::checkReferee(const ros::Time &time) {
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_ && !chassis_output_) {
    ROS_INFO("Chassis output ON");
    chassisOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_ && !gimbal_output_) {
    ROS_INFO("Gimbal output ON");
    gimbalOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_ && !shooter_output_) {
    ROS_INFO("Shooter output ON");
    shooterOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_) chassis_output_ = true;
  else chassis_output_ = false;
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_) gimbal_output_ = true;
  else gimbal_output_ = false;
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_) shooter_output_ = true;
  else shooter_output_ = false;
}

}