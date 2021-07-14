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
  };
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(-1.);
    if (data_->pos_x_ == 0 || data_->pos_x_ == move_distance_) vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  double move_distance_;
  double scale_x_;
};
}

#endif //RM_FSM_STATE_STANDBY_H_
