//
// Created by astro on 2020/12/8.
//

#ifndef RM_FSM_STATE_PASSIVE_H
#define RM_FSM_STATE_PASSIVE_H
#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StatePassive : public State {
 public:
  StatePassive(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string) :
      State(nh, fsm_data, state_string) {}
  void onEnter() override { ROS_INFO("Enter passive mode"); }
  void run() override {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::PASSIVE);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PASSIVE);
    sendCommand(ros::Time::now());
  };
  void onExit() override { ROS_INFO("Exit passive mode"); }
};
}

#endif //RM_FSM_STATE_PASSIVE_H
