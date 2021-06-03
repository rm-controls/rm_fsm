//
// Created by peter on 2020/12/3.
//

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
State::State(ros::NodeHandle &nh, Data *fsm_data, std::string state_name)
    : nh_(nh), data_(fsm_data), state_name_(std::move(state_name)) {
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new ChassisCommandSender(chassis_nh, *data_->referee_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_2d_cmd_sender_ = new Vel2DCommandSender(vel_nh);
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh, *data_->referee_);
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new ShooterCommandSender(shooter_nh, *data_->referee_);
};

Fsm::Fsm(ros::NodeHandle &nh) : nh_(nh) {
  ros::NodeHandle ctrl_handle(nh, "controller_manager");
  controller_manager_ = new ControllerManager(ctrl_handle);
  controller_manager_->loadAllControllers();
  controller_manager_->startInformationControllers();
  data_.init(nh);
  string2state.insert(std::make_pair("invalid", nullptr));
  current_state_ = string2state["invalid"];
  next_state_ = current_state_;
  operating_mode_ = NORMAL;
}

void Fsm::run() {
  checkSwitch(ros::Time::now());
  data_.referee_->read();
  if (operating_mode_ == NORMAL) {
    next_state_name_ = getDesiredState();
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

void Fsm::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.1) {
    remoteControlTurnOff();
    remote_is_open_ = false;
    ROS_INFO("Remote off");
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.1) {
    remoteControlTurnOn();
    remote_is_open_ = true;
    ROS_INFO("Remote on");
  }
}
}