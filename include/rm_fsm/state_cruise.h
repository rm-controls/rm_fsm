//
// Created by peter on 2021/8/3.
//

#ifndef RM_FSM_INCLUDE_STATE_CRUISE_H_
#define RM_FSM_INCLUDE_STATE_CRUISE_H_

#include <cstdlib>
#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class StateCruise : public StateAttack {
 public:
  StateCruise(ros::NodeHandle &nh, Data *data) : StateAttack(nh, data), start_pos_(0.), start_flag_(false) {
    state_name_ = "CRUISE";
    random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
  }
 protected:
  void setChassis() override {
    StateBase::setChassis();
    if (vel_2d_cmd_sender_->getMsg()->linear.x == 0.) vel_2d_cmd_sender_->setLinearXVel(1.);
    if (data_->pos_x_ <= start_pos_ - random_distance_ || data_->pos_x_ <= -move_distance_) {
      vel_2d_cmd_sender_->setLinearXVel(1.);
      if (!start_flag_) {
        start_pos_ = data_->pos_x_;
        random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
        start_flag_ = true;
      }
    } else if (data_->pos_x_ >= start_pos_ + random_distance_ || data_->pos_x_ >= 0.) {
      vel_2d_cmd_sender_->setLinearXVel(-1.);
      if (!start_flag_) {
        start_pos_ = data_->pos_x_;
        random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
        start_flag_ = true;
      }
    } else start_flag_ = false;
    ROS_INFO("pos %f", data_->pos_x_);
  }
  void setGimbal(SideCommandSender *side_cmd_sender) override {
    if (side_cmd_sender->pos_yaw_ >= side_cmd_sender->yaw_max_) side_cmd_sender->yaw_direct_ = -1.;
    else if (side_cmd_sender->pos_yaw_ <= side_cmd_sender->yaw_min_) side_cmd_sender->yaw_direct_ = 1.;
    if (side_cmd_sender->pos_pitch_ >= side_cmd_sender->pitch_max_) side_cmd_sender->pitch_direct_ = -1.;
    else if (side_cmd_sender->pos_pitch_ <= side_cmd_sender->pitch_min_) side_cmd_sender->pitch_direct_ = 1.;
    setTrack(side_cmd_sender);
  }
  void setShooter(SideCommandSender *side_cmd_sender) override {
    if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
      side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      side_cmd_sender->shooter_cmd_sender_->checkError(side_cmd_sender->gimbal_des_error_, ros::Time::now());
    } else side_cmd_sender->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
 private:
  double random_distance_, start_pos_;
  bool start_flag_;
};
}

#endif //RM_FSM_INCLUDE_STATE_CRUISE_H_
