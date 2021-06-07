//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_ATTACK_H_
#define RM_FSM_STATE_ATTACK_H_

namespace rm_fsm {
class StateAttack : public StateBase {
 public:
  StateAttack(ros::NodeHandle &nh, Data *data, const std::string &state_string)
      : StateBase(nh, data, state_string) {
    ros::NodeHandle move_nh = ros::NodeHandle(nh, "auto/move");
    if (!move_nh.getParam("scale_x", scale_x_)) {
      ROS_ERROR("Scale x no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!move_nh.getParam("scale_yaw", scale_yaw_)) {
      ROS_ERROR("Scale yaw no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!move_nh.getParam("scale_pitch", scale_pitch_)) {
      ROS_ERROR("Scale pitch no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  }
 protected:
  void setChassis() override {
    if (move_status_ == LEAVE_START || move_status_ == LEAVE_END)
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    else if (move_status_ == APPROACH_START || move_status_ == APPROACH_END) {
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
    }
    if (move_status_ == LEAVE_START) vel_2d_cmd_sender_->setLinearXVel(scale_x_);
    else if (move_status_ == LEAVE_END) vel_2d_cmd_sender_->setLinearXVel(-scale_x_);
  }
  void setGimbal() override {
    gimbal_cmd_sender_->setRate(scale_yaw_, scale_pitch_);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::PASSIVE);
    gimbal_cmd_sender_->updateCost(data_->track_data_array_, false);
  }
  void setShooter() override {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PASSIVE);
    shooter_cmd_sender_->checkGimbalError(data_->gimbal_des_error_.error);
  }
  double scale_yaw_, scale_pitch_, scale_x_;
};
}

#endif //RM_FSM_STATE_ATTACK_H_
