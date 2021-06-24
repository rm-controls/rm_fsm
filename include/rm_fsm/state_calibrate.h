//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_CALIBRATE_H_
#define RM_FSM_STATE_CALIBRATE_H_

#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StateCalibrate : public StateBase {
 public:
  StateCalibrate(ros::NodeHandle &nh, Data *data, const std::string &state_string)
      : StateBase(nh, data, state_string) {
    ros::NodeHandle calibrate_nh = ros::NodeHandle(nh, "auto/calibrate");
    if (!calibrate_nh.getParam("scale_x", scale_x_)) {
      ROS_ERROR("Move speed no defined (namespace: %s)", nh.getNamespace().c_str());
    }
  }
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(-scale_x_);
  }
  double scale_x_;
};
}

#endif //RM_FSM_STATE_CALIBRATE_H_
