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
    if (!auto_nh.getParam("stop_distance", stop_distance_))
      ROS_ERROR("Stop distance no defined (namespace: %s)", nh.getNamespace().c_str());
  };
 protected:
  void setChassis() override {
    StateBase::setChassis();
    if (data_->pos_x_ > -(move_distance_ - stop_distance_)) vel_2d_cmd_sender_->setLinearXVel(-1.);
    else vel_2d_cmd_sender_->setLinearXVel(0.);
  }
  void setGimbal(SideCommandSender *side_cmd_sender) override {
    StateBase::setGimbal(side_cmd_sender);
    side_cmd_sender->gimbal_cmd_sender_->setZero();
  }
  double move_distance_{}, stop_distance_{};
};
}

#endif //RM_FSM_STATE_STANDBY_H_
