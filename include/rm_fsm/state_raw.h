//
// Created by astro on 2020/12/8.
//

#ifndef RM_FSM_STATE_RAW_H
#define RM_FSM_STATE_RAW_H
#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateRaw : public State {
 public:
  StateRaw(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string)
      : State(nh, fsm_data, state_string) {}
  void onEnter() override { ROS_INFO("Enter raw mode"); };
  void run() override {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    vel_cmd_sender_->setXVel(data_->dbus_data_.ch_r_y);
    vel_cmd_sender_->setYVel(-data_->dbus_data_.ch_r_x);
    vel_cmd_sender_->setWVel(data_->dbus_data_.wheel);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    gimbal_cmd_sender_->setRate(-data_->dbus_data_.ch_l_x, -data_->dbus_data_.ch_l_y);
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::UP) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    else if (data_->dbus_data_.s_l == rm_msgs::DbusData::MID) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    sendCommand(ros::Time::now());
  };
  void onExit() override { ROS_INFO("Exit raw mode"); };
};
}

#endif //RM_FSM_STATE_RAW_H
