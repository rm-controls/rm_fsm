//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_FSM_SENTRY_H_
#define RM_FSM_FSM_SENTRY_H_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_calibrate.h"
#include "rm_fsm/state_attack.h"
#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class FsmSentry : public FsmBase {
 public:
  FsmSentry(ros::NodeHandle &nh) : FsmBase(nh) {
    string2state.insert(std::pair<std::string, StateBase *>(state_raw_->getName(), state_raw_));
    string2state.insert(std::pair<std::string, StateBase *>(state_calibrate_->getName(), state_calibrate_));
    string2state.insert(std::pair<std::string, StateBase *>(state_attack_->getName(), state_attack_));
    string2state.insert(std::pair<std::string, StateBase *>(state_standby_->getName(), state_standby_));
    string2state.insert(std::pair<std::string, StateBase *>(state_idle_->getName(), state_idle_));
    current_state_ = string2state["RAW"];
  }
 protected:
  std::string getNextState() override {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
      if (!state_calibrate_->getCalibrateStatus()) return "CALIBRATE";
      sendMode(ros::Time::now());
      if (data_.referee_.referee_data_.interactive_data.header_data_.data_cmd_id_ == 0x0200
          && data_.referee_.referee_data_.interactive_data.data_ == 0)
        return "STANDBY";
      else return "ATTACK";
    } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) return "RAW";
    else return "IDLE";
  }
  void sendMode(const ros::Time &time) {
    if (time - last_send_ < ros::Duration(0.5)) return;
    int receiver_id = data_.referee_.robot_id_ == rm_common::RobotId::RED_SENTRY ? rm_common::RobotId::RED_STANDARD_4
                                                                                 : rm_common::RobotId::BLUE_STANDARD_4;
    data_.referee_.sendInteractiveData(0x0201, receiver_id, data_.referee_.referee_data_.interactive_data.data_);
    last_send_ = time;
  }
  ros::Time last_send_ = ros::Time::now();
 private:
  StateBase *state_idle_ = new StateBase(nh_, &data_, "IDLE");
  StateRaw *state_raw_ = new StateRaw(nh_, &data_);
  StateCalibrate *state_calibrate_ = new StateCalibrate(nh_, &data_);
  StateStandby *state_standby_ = new StateStandby(nh_, &data_);
  StateAttack *state_attack_ = new StateAttack(nh_, &data_);
};
}

#endif //RM_FSM_FSM_SENTRY_H_