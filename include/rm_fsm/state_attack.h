//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_ATTACK_H_
#define RM_FSM_STATE_ATTACK_H_

#include "rm_fsm/state_standby.h"
namespace rm_fsm {
class StateAttack : public StateStandby {
 public:
  StateAttack(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string)
      : StateStandby(nh, fsm_data, state_string) {
    ros::NodeHandle move_nh = ros::NodeHandle(nh, "auto/move");
    if (!move_nh.getParam("scale_yaw", scale_yaw_)) {
      ROS_ERROR("Scale yaw no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!move_nh.getParam("scale_pitch", scale_pitch_)) {
      ROS_ERROR("Scale pitch no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  }
 protected:
  void setGimbal() override {
    gimbal_cmd_sender_->setRate(scale_yaw_, scale_pitch_);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_->track_data_array_, false);
  }
  void setShooter() override {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkGimbalError(data_->gimbal_des_error_.error);
  }
  double scale_yaw_, scale_pitch_;
};
}

#endif //RM_FSM_STATE_ATTACK_H_
