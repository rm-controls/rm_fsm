//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_ATTACK_H_
#define RM_FSM_STATE_ATTACK_H_

namespace rm_fsm {
class StateAttack : public State {
 public:
  StateAttack(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string)
      : State(nh, fsm_data, state_string) {
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("collision", collision_)) {
      ROS_ERROR("Collision no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    ros::NodeHandle move_nh = ros::NodeHandle(auto_nh, "move");
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
  void updatePosition(int position) { position_ = position; }
  void setChassis() override {
    if (position_ == 1) vel_2d_cmd_sender_->setLinearXVel(scale_x_);
    else if (position_ == 3) vel_2d_cmd_sender_->setLinearXVel(-scale_x_);

    if (position_ == 1 || position_ == 3) chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    else if (position_ == 2 || position_ == 4) {
      if (collision_) chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
      else chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    }
  }
  void setGimbal() override {
    gimbal_cmd_sender_->setRate(scale_yaw_, scale_pitch_);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_->track_data_array_, false);
  }
  void setShooter() override {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkGimbalError(data_->gimbal_des_error_.error);
  }
  double scale_yaw_, scale_pitch_, scale_x_;
  int position_;
  bool collision_;
};
}

#endif //RM_FSM_STATE_ATTACK_H_
