//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_ATTACK_H_
#define RM_FSM_STATE_ATTACK_H_

#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class StateAttack : public StateStandby {
 public:
  StateAttack(ros::NodeHandle &nh, Data *data) : StateStandby(nh, data) {
    state_name_ = "ATTACK";
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("stop_distance", stop_distance_)) {
      ROS_ERROR("Stop distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  }
 protected:
  void setChassis() override {
    StateStandby::setChassis();
    updateMoveStatus();
    if (move_status_ == LEAVE_START) vel_2d_cmd_sender_->setLinearXVel(1.);
    else if (move_status_ == LEAVE_END) vel_2d_cmd_sender_->setLinearXVel(-1.);
  }
 private:
  void updateMoveStatus() {
    if (move_status_ == LEAVE_START && data_->pos_x_ >= move_distance_ - stop_distance_) move_status_ = LEAVE_END;
    else if (move_status_ == LEAVE_END && data_->pos_x_ <= stop_distance_) move_status_ = LEAVE_START;
  }
  enum MoveStatus { LEAVE_START, LEAVE_END };
  MoveStatus move_status_ = LEAVE_START;
  double stop_distance_{};
};
}

#endif //RM_FSM_STATE_ATTACK_H_
