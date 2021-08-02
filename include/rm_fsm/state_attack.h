//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_ATTACK_H_
#define RM_FSM_STATE_ATTACK_H_

#include <cstdlib>
#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class StateAttack : public StateStandby {
 public:
  StateAttack(ros::NodeHandle &nh, Data *data) : StateStandby(nh, data) {
    state_name_ = "ATTACK";
    ros::NodeHandle attack_nh = ros::NodeHandle(nh, "upper/attack");
    if (!attack_nh.getParam("pitch", expect_pitch_))
      ROS_ERROR("pitch no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!attack_nh.getParam("yaw", expect_yaw_))
      ROS_ERROR("yaw no defined (namespace: %s)", nh.getNamespace().c_str());
  }
  bool getAttackStatus() { return attack_finish_; }
 protected:
  void setChassis() override {
    StateBase::setChassis();
    if (data_->pos_x_ < -stop_distance_) vel_2d_cmd_sender_->setLinearXVel(1.);
    else vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  void setGimbal(SideCommandSender *side_cmd_sender) override {
    if (side_cmd_sender == upper_cmd_sender_) {
      if (expect_yaw_ > 0.) {
        if (side_cmd_sender->pos_yaw_ < expect_yaw_ * 0.95) side_cmd_sender->yaw_direct_ = 1;
        else if (side_cmd_sender->pos_yaw_ > expect_yaw_ * 1.05) side_cmd_sender->yaw_direct_ = -1;
        else side_cmd_sender->yaw_direct_ = 0.;
      } else {
        if (side_cmd_sender->pos_yaw_ > expect_yaw_ * 0.95) side_cmd_sender->yaw_direct_ = -1;
        else if (side_cmd_sender->pos_yaw_ < expect_yaw_ * 1.05) side_cmd_sender->yaw_direct_ = 1;
        else side_cmd_sender->yaw_direct_ = 0.;
      }
      if (expect_pitch_ > 0.) {
        if (side_cmd_sender->pos_pitch_ < expect_pitch_ * 0.95) side_cmd_sender->pitch_direct_ = 1;
        else if (side_cmd_sender->pos_pitch_ > expect_pitch_ * 1.05) side_cmd_sender->pitch_direct_ = -1;
        else side_cmd_sender->pitch_direct_ = 0.;
      } else {
        if (side_cmd_sender->pos_pitch_ > expect_pitch_ * 0.95) side_cmd_sender->pitch_direct_ = -1;
        else if (side_cmd_sender->pos_pitch_ < expect_pitch_ * 1.05) side_cmd_sender->pitch_direct_ = 1;
        else side_cmd_sender->pitch_direct_ = 0.;
      }
      side_cmd_sender->gimbal_cmd_sender_->setRate(side_cmd_sender->yaw_direct_ * 0.05,
                                                   side_cmd_sender->pitch_direct_ * 0.05);
    } else {
      if (side_cmd_sender->pos_yaw_ >= side_cmd_sender->yaw_max_) side_cmd_sender->yaw_direct_ = -1.;
      else if (side_cmd_sender->pos_yaw_ <= side_cmd_sender->yaw_min_) side_cmd_sender->yaw_direct_ = 1.;
      if (side_cmd_sender->pos_pitch_ >= side_cmd_sender->pitch_max_) side_cmd_sender->pitch_direct_ = -1.;
      else if (side_cmd_sender->pos_pitch_ <= side_cmd_sender->pitch_min_) side_cmd_sender->pitch_direct_ = 1.;
      setTrack(side_cmd_sender);
    }
  }
  void setShooter(SideCommandSender *side_cmd_sender) override {
    if (side_cmd_sender == upper_cmd_sender_) {
      if (data_->referee_.referee_data_.game_status_.game_progress_ == 4
          && side_cmd_sender->gimbal_cmd_sender_->getMsg()->rate_yaw == 0.
          && side_cmd_sender->gimbal_cmd_sender_->getMsg()->rate_pitch == 0.) {
        if (data_->referee_.referee_data_.bullet_remaining_.bullet_remaining_num_17_mm_ > 400
            && ((data_->referee_.referee_data_.robot_color_ == "blue"
                && data_->referee_.referee_data_.game_robot_hp_.blue_outpost_hp_ > 0)
                || (data_->referee_.referee_data_.robot_color_ == "red"
                    && data_->referee_.referee_data_.game_robot_hp_.red_outpost_hp_ > 0)))
          side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        else attack_finish_ = true;
      }
    } else { // lower gimbal auto track
      if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
        side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        side_cmd_sender->shooter_cmd_sender_->checkError(side_cmd_sender->gimbal_des_error_, ros::Time::now());
      } else side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    }
  }
 private:
  double expect_pitch_{}, expect_yaw_{};
  bool attack_finish_{false};
};
}

#endif //RM_FSM_STATE_ATTACK_H_
