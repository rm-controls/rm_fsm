//
// Created by peter on 2021/7/14.
//

#ifndef RM_FSM_STATE_STANDBY_H_
#define RM_FSM_STATE_STANDBY_H_

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateStandby : public StateBase {
 public:
  StateStandby(ros::NodeHandle &nh, Data *data) : StateBase(nh, data, "STANDBY") {
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("move_distance", move_distance_))
      ROS_ERROR("Move distance no defined (namespace: %s)", nh.getNamespace().c_str());
  };
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(-1.);
    if (data_->pos_x_ - move_distance_ * 0.1 <= 0.) vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  void setGimbal(SideCommandSender *side_cmd_sender) override {
    double yaw_max = side_cmd_sender->yaw_max_;
    if (state_name_ == "STANDBY") yaw_max = 0.;
    if (side_cmd_sender->pos_yaw_ >= yaw_max) side_cmd_sender->yaw_direct_ = -1.;
    else if (side_cmd_sender->pos_yaw_ <= side_cmd_sender->yaw_min_) side_cmd_sender->yaw_direct_ = 1.;
    if (side_cmd_sender->pos_pitch_ >= side_cmd_sender->pitch_max_) side_cmd_sender->pitch_direct_ = -1.;
    else if (side_cmd_sender->pos_pitch_ <= side_cmd_sender->pitch_min_) side_cmd_sender->pitch_direct_ = 1.;
    side_cmd_sender->gimbal_cmd_sender_->setRate(side_cmd_sender->yaw_direct_, side_cmd_sender->pitch_direct_);
    side_cmd_sender->gimbal_cmd_sender_->setBulletSpeed(side_cmd_sender->shooter_cmd_sender_->getSpeed());
    side_cmd_sender->gimbal_cmd_sender_->updateCost(side_cmd_sender->track_data_);
  }
  void setShooter(SideCommandSender *side_cmd_sender) override {
    if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
      side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      side_cmd_sender->shooter_cmd_sender_->checkError(side_cmd_sender->gimbal_des_error_, ros::Time::now());
    } else
      side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
 protected:
  double move_distance_{};
};
}

#endif //RM_FSM_STATE_STANDBY_H_
