//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_FSM_SENTRY_H_
#define RM_FSM_FSM_SENTRY_H_

#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_passive.h"
#include "rm_fsm/state_automatic.h"

namespace rm_fsm {
class FsmSentry : public Fsm {
 public:
  explicit FsmSentry(ros::NodeHandle &nh) : Fsm(nh) {
    state_passive_ = new StatePassive(nh, &this->data_, "passive");
    state_raw_ = new StateRaw(nh, &this->data_, "raw");
    state_automatic_ = new StateAutomatic(nh, &this->data_, "automatic");

    this->string2state.insert(std::pair<std::string, State *>("passive", state_passive_));
    this->string2state.insert(std::pair<std::string, State *>("raw", state_raw_));
    this->string2state.insert(std::pair<std::string, State *>("automatic", state_automatic_));
    this->current_state_ = this->string2state["passive"];
  };
  std::string getDesiredState() {
    if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) return "passive";
    else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::MID) return "raw";
    else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::UP) return "automatic";
    else return "passive";
  };
  StatePassive *state_passive_;
  StateRaw *state_raw_;
  StateAutomatic *state_automatic_;
};
}

#endif //RM_FSM_FSM_SENTRY_H_
