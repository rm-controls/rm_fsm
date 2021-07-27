//
// Created by astro on 2020/12/8.
//

#ifndef RM_FSM_STATE_RAW_H
#define RM_FSM_STATE_RAW_H
#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateRaw : public StateBase {
 public:
  StateRaw(ros::NodeHandle &nh, Data *data) : StateBase(nh, data, "RAW") {};
 protected:
  void setChassis() override {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    vel_2d_cmd_sender_->setLinearXVel(data_->dbus_data_.ch_r_y);
  }
  void setUpperGimbal() override {
    upper_gimbal_cmd_sender_->setRate(-data_->dbus_data_.ch_l_x, -data_->dbus_data_.ch_l_y);
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::DOWN)
      upper_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    else {
      upper_gimbal_cmd_sender_->setBulletSpeed(upper_shooter_cmd_sender_->getSpeed());
      upper_gimbal_cmd_sender_->updateCost(data_->upper_track_data_array_);
    }
  }
  void setLowerGimbal() override {
    lower_gimbal_cmd_sender_->setRate(-data_->dbus_data_.ch_l_x, -data_->dbus_data_.ch_l_y);
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::DOWN)
      lower_gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    else {
      lower_gimbal_cmd_sender_->setBulletSpeed(lower_shooter_cmd_sender_->getSpeed());
      lower_gimbal_cmd_sender_->updateCost(data_->lower_track_data_array_);
    }
  }
  void setUpperShooter() override {
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      upper_shooter_cmd_sender_->checkError(data_->upper_gimbal_des_error_, ros::Time::now());
    } else if (data_->dbus_data_.s_l == rm_msgs::DbusData::MID) upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void setLowerShooter() override {
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      lower_shooter_cmd_sender_->checkError(data_->lower_gimbal_des_error_, ros::Time::now());
    } else if (data_->dbus_data_.s_l == rm_msgs::DbusData::MID) lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
};
}

#endif //RM_FSM_STATE_RAW_H
