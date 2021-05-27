//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_STANDBY_H_
#define RM_FSM_STATE_STANDBY_H_

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateStandby : public State {
 public:
  StateStandby(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string)
      : State(nh, fsm_data, state_string) {
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("collision", collision_)) {
      ROS_ERROR("Collision no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    ros::NodeHandle move_nh = ros::NodeHandle(auto_nh, "move");
    if (!move_nh.getParam("scale_x", scale_x_)) {
      ROS_ERROR("Scale x no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  }
 protected:
  void setChassis() override {
    if (position_ == 1) vel_2d_cmd_sender_->setLinearXVel(scale_x_);
    else if (position_ == 3) vel_2d_cmd_sender_->setLinearXVel(-scale_x_);

    if (position_ == 1 || position_ == 3) chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    else if (position_ == 2 || position_ == 4) {
      if (collision_) chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
      else chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    }
  }
  void updatePosition(int position) { position_ = position; }
  double scale_x_;
  int position_;
  bool collision_;
};
}

#endif //RM_FSM_STATE_STANDBY_H_
