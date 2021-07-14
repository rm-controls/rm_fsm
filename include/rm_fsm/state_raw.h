//
// Created by astro on 2020/12/8.
//

#ifndef RM_FSM_STATE_RAW_H
#define RM_FSM_STATE_RAW_H
#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateRaw : public StateBase {
 public:
  StateRaw(ros::NodeHandle &nh, Data *data) : StateBase(nh, data, "RAW") {}
 protected:
  void setChassis() override {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    vel_2d_cmd_sender_->setLinearXVel(data_->dbus_data_.ch_r_y);
  }
  void setGimbal() override {
    gimbal_cmd_sender_->setRate(-data_->dbus_data_.ch_l_x, -data_->dbus_data_.ch_l_y);
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::DOWN) { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
    else {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
      gimbal_cmd_sender_->updateCost(data_->track_data_array_);
    }
  }
  void setShooter() override {
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(data_->gimbal_des_error_, ros::Time::now());
    } else if (data_->dbus_data_.s_l == rm_msgs::DbusData::MID) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
};
}

#endif //RM_FSM_STATE_RAW_H
