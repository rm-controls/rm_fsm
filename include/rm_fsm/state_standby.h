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
    if (!auto_nh.getParam("move_distance", move_distance_)) {
      ROS_ERROR("Move distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    ros::NodeHandle attack_nh = ros::NodeHandle(auto_nh, "attack");
    if (!attack_nh.getParam("scale_x", scale_x_)) {
      ROS_ERROR("Scale x no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    ros::NodeHandle pitch_nh = ros::NodeHandle(attack_nh, "pitch");
    if (!pitch_nh.getParam("scale", scale_pitch_)) {
      ROS_ERROR("Scale no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!pitch_nh.getParam("max", pitch_max_)) {
      ROS_ERROR("Max no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!pitch_nh.getParam("min", pitch_min_)) {
      ROS_ERROR("Min no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    ros::NodeHandle yaw_nh = ros::NodeHandle(attack_nh, "yaw");
    if (!yaw_nh.getParam("scale", scale_yaw_)) {
      ROS_ERROR("Scale no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!yaw_nh.getParam("max", yaw_max_)) {
      ROS_ERROR("Max no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!yaw_nh.getParam("min", yaw_min_)) {
      ROS_ERROR("Min no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  };
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(scale_x_);
    if (data_->pos_x_ - move_distance_ * 0.1 <= 0 || data_->pos_x_ + move_distance_ * 0.1 >= move_distance_)
      vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  void setGimbal() override {
    if (data_->pos_yaw_ > yaw_max_) direct_yaw_ = -1;
    else if (data_->pos_yaw_ < yaw_min_) direct_yaw_ = 1;
    if (data_->pos_pitch_ > pitch_max_) direct_pitch_ = -1;
    else if (data_->pos_pitch_ < pitch_min_) direct_pitch_ = 1;
    gimbal_cmd_sender_->setRate(scale_yaw_ * direct_yaw_, scale_pitch_ * direct_pitch_);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    gimbal_cmd_sender_->updateCost(data_->track_data_array_);
  }
  void setShooter() override {
    if (gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK)
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    else
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    shooter_cmd_sender_->checkError(data_->gimbal_des_error_, ros::Time::now());
  }
  double move_distance_{};
  double scale_x_{}, scale_yaw_{}, scale_pitch_{};
  double pitch_max_{}, pitch_min_{}, yaw_max_{}, yaw_min_{};
  int direct_pitch_ = 1, direct_yaw_ = 1;
};
}

#endif //RM_FSM_STATE_STANDBY_H_
