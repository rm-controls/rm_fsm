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
  void setGimbal(SideCommandSender *side_cmd_sender) override {
    side_cmd_sender->gimbal_cmd_sender_->setRate(-data_->dbus_data_.ch_l_x, -data_->dbus_data_.ch_l_y);
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::DOWN)
      side_cmd_sender->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    else {
      side_cmd_sender->gimbal_cmd_sender_->setBulletSpeed(side_cmd_sender->shooter_cmd_sender_->getSpeed());
      side_cmd_sender->gimbal_cmd_sender_->updateCost(side_cmd_sender->track_data_);
    }
  }
  void setShooter(SideCommandSender *side_cmd_sender) override {
    if (data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      side_cmd_sender->shooter_cmd_sender_->checkError(side_cmd_sender->gimbal_des_error_, ros::Time::now());
    } else if (data_->dbus_data_.s_l == rm_msgs::DbusData::MID)
      side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
};
}

#endif //RM_FSM_STATE_RAW_H
