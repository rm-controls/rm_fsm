//
// Created by luotinkai on 2022/6/5.
//

#include "rm_fsm/common/state_base.h"

StateBase::StateBase(ros::NodeHandle &nh) : controller_manager_(nh) {}

void StateBase::sendCommand(const ros::Time &time) {
  vel_2d_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
}

void StateBase::setChassis() {}

void StateBase::setGimbal() {}

void StateBase::setShooter() {}

void StateBase::checkReferee(const ros::Time &time) {
  if (fsm_data_.game_robot_status_.mains_power_chassis_output_ &&
      !chassis_output_) {
    ROS_INFO("Chassis output ON");
    chassisOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_gimbal_output_ &&
      !gimbal_output_) {
    ROS_INFO("Gimbal output ON");
    gimbalOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_shooter_output_ &&
      !shooter_output_) {
    ROS_INFO("Shooter output ON");
    shooterOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_chassis_output_)
    chassis_output_ = true;
  else
    chassis_output_ = false;
  if (fsm_data_.game_robot_status_.mains_power_gimbal_output_)
    gimbal_output_ = true;
  else
    gimbal_output_ = false;
  if (fsm_data_.game_robot_status_.mains_power_shooter_output_)
    shooter_output_ = true;
  else
    shooter_output_ = false;
}

void StateBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - fsm_data_.dbus_data_.stamp).toSec() > 0.3) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - fsm_data_.dbus_data_.stamp).toSec() < 0.3) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
}

void StateBase::chassisOutputOn() { ROS_INFO("Chassis output ON"); }

void StateBase::gimbalOutputOn() { ROS_INFO("Gimbal output ON"); }

void StateBase::shooterOutputOn() { ROS_INFO("Shooter output ON"); }

void StateBase::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
}

void StateBase::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
  controller_manager_.stopCalibrationControllers();
}

void StateBase::run() {
  fsm_data_.update(ros::Time::now());
  checkReferee(ros::Time::now());
  checkSwitch(ros::Time::now());
  setGimbal();
  setShooter();
  sendCommand(ros::Time::now());
  controller_manager_.update();
}
