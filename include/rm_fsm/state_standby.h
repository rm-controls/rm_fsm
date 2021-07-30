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
    ros::NodeHandle standby_nh = ros::NodeHandle(auto_nh, "standby");
    ros::NodeHandle upper_nh = ros::NodeHandle(standby_nh, "upper");
    getGimbalParam(upper_nh, "upper");
    ros::NodeHandle lower_nh = ros::NodeHandle(standby_nh, "lower");
    getGimbalParam(lower_nh, "lower");
  };
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(1.);
    if (data_->pos_x_ - move_distance_ * 0.1 <= 0 || data_->pos_x_ + move_distance_ * 0.1 >= move_distance_)
      vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  void setUpperGimbal() override {
    if (data_->upper_yaw_ >= upper_yaw_max_) upper_scale_yaw_ = -1.;
    else if (data_->upper_yaw_ <= upper_yaw_min_) upper_scale_yaw_ = 1.;
    if (data_->upper_pitch_ >= upper_pitch_max_) upper_scale_pitch_ = -1.;
    else if (data_->upper_pitch_ <= upper_pitch_min_) upper_scale_pitch_ = 1.;
    upper_gimbal_cmd_sender_->setRate(upper_scale_yaw_, upper_scale_pitch_);
    upper_gimbal_cmd_sender_->setBulletSpeed(upper_shooter_cmd_sender_->getSpeed());
    upper_gimbal_cmd_sender_->updateCost(data_->upper_track_data_array_);
  }
  void setLowerGimbal() override {
    if (data_->lower_yaw_ >= lower_yaw_max_) lower_scale_yaw_ = -1.;
    else if (data_->lower_yaw_ <= lower_yaw_min_) lower_scale_yaw_ = 1.;
    if (data_->lower_pitch_ >= lower_pitch_max_) lower_scale_pitch_ = -1.;
    else if (data_->lower_pitch_ <= lower_pitch_min_) lower_scale_pitch_ = 1.;
    lower_gimbal_cmd_sender_->setRate(lower_scale_yaw_, lower_scale_pitch_);
    lower_gimbal_cmd_sender_->setBulletSpeed(lower_shooter_cmd_sender_->getSpeed());
    lower_gimbal_cmd_sender_->updateCost(data_->lower_track_data_array_);
  }
  void setUpperShooter() override {
    if (upper_gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
      upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      upper_shooter_cmd_sender_->checkError(data_->upper_gimbal_des_error_, ros::Time::now());
    } else
      upper_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void setLowerShooter() override {
    if (lower_gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
      lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      lower_shooter_cmd_sender_->checkError(data_->lower_gimbal_des_error_, ros::Time::now());
    } else
      lower_shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
 protected:
  void getGimbalParam(ros::NodeHandle &nh, const std::string &side) {
    XmlRpc::XmlRpcValue pitch_value, yaw_value;
    nh.getParam("pitch", pitch_value);
    try {
      if (side == "upper") {
        upper_pitch_min_ = (double) (pitch_value[0]);
        upper_pitch_max_ = (double) (pitch_value[1]);
      } else {
        lower_pitch_min_ = (double) (pitch_value[0]);
        lower_pitch_max_ = (double) (pitch_value[1]);
      }
    } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("%s", e.getMessage().c_str()); }
    nh.getParam("yaw", yaw_value);
    try {
      if (side == "upper") {
        upper_yaw_min_ = (double) (yaw_value[0]);
        upper_yaw_max_ = (double) (yaw_value[1]);
      } else {
        lower_yaw_min_ = (double) (yaw_value[0]);
        lower_yaw_max_ = (double) (yaw_value[1]);
      }
    } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("%s", e.getMessage().c_str()); }
  }
  double move_distance_{};
  double upper_pitch_max_{}, upper_pitch_min_{}, upper_yaw_max_{}, upper_yaw_min_{};
  double lower_pitch_max_{}, lower_pitch_min_{}, lower_yaw_max_{}, lower_yaw_min_{};
  double upper_scale_pitch_ = 1., upper_scale_yaw_ = 1.;
  double lower_scale_pitch_ = 1., lower_scale_yaw_ = 1.;
};
}

#endif //RM_FSM_STATE_STANDBY_H_
