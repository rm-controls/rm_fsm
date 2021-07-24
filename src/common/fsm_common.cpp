//
// Created by peter on 2020/12/3.
//

#include "rm_fsm/common/fsm_common.h"
#include "rm_common/decision/controller_manager.h"

namespace rm_fsm {
FsmBase::FsmBase(ros::NodeHandle &nh) : nh_(nh), data_(nh) ,controller_manager_(nh),calibration_loader(nh){
  controller_manager_.startStateControllers();
  calibration_loader.startCalibrationControllers();

  string2state.insert(std::make_pair("INVALID", nullptr));
  current_state_ = string2state["INVALID"];

}

void FsmBase::run() {
  ros::Time time = ros::Time::now();
  data_.update(time);
  checkReferee(time);
  checkSwitch(time);
  controller_manager_.update();
  std::string next_state_name = getNextState();
  if (next_state_name != current_state_->getName()) {
    current_state_ = string2state[next_state_name];
    current_state_->onEnter();
    current_state_->run();
  } else current_state_->run();

}

void FsmBase::checkReferee(const ros::Time &time) {
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_) chassis_output_ = true;
  else chassis_output_ = false;
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_) gimbal_output_ = true;
  else gimbal_output_ = false;
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_) shooter_output_ = true;
  else shooter_output_ = false;
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_ && chassis_output_){
    ROS_INFO("Chassis output ON");
    chassisOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_ && gimbal_output_){
    ROS_INFO("Gimbal output ON");
    gimbalOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_ && shooter_output_){
    ROS_INFO("Shooter output ON");
    shooterOutputOn();
  }
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
}