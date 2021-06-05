//
// Created by peter on 2020/12/3.
//

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
FsmBase::FsmBase(ros::NodeHandle &nh) : nh_(nh), data_(nh) {
  controller_loader_ = new ControllerLoader(nh);
  controller_loader_->loadControllers();
  calibration_manager_ = new CalibrationManager(nh);
  ros::NodeHandle state_ctrl_nh(nh, "state_controllers_switch");
  switch_state_ctrl_srv_ = new SwitchControllersService(state_ctrl_nh);
  switch_state_ctrl_srv_->startControllersOnly();
  switch_state_ctrl_srv_->callService();
  ros::NodeHandle base_ctrl_nh(nh, "base_controllers_switch");
  switch_base_ctrl_srv_ = new SwitchControllersService(base_ctrl_nh);
  string2state.insert(std::make_pair("invalid", nullptr));
  current_state_ = string2state["invalid"];
  next_state_ = current_state_;
  operating_mode_ = NORMAL;
}

void FsmBase::run() {
  ros::Time time = ros::Time::now();
  checkSwitch(time);
  calibration_manager_->checkCalibrate(time);
  data_.referee_.read();
  if (operating_mode_ == NORMAL) {
    next_state_name_ = getNextState();
    if (next_state_name_ != current_state_->getName()) {
      operating_mode_ = TRANSITIONING;
      next_state_ = string2state[next_state_name_];
    } else current_state_->run();
  }
  if (operating_mode_ == TRANSITIONING) {
    current_state_->onExit();
    current_state_ = next_state_;
    current_state_->onEnter();
    operating_mode_ = NORMAL;
  }
}

void FsmBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.1) {
    ROS_INFO("Remote off");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.1) {
    ROS_INFO("Remote on");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
}
}