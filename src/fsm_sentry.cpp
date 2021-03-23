//
// Created by astro on 2021/1/26.
//

#include "rm_fsm/fsm_sentry.h"
template<typename T>
FsmSentry<T>::FsmSentry(ros::NodeHandle &node_handle) : Fsm<T>(node_handle) {
  state_passive_ = new StatePassive<T>(&this->data_, "passive", node_handle);
  state_raw_ = new StateRaw<T>(&this->data_, "raw", node_handle);
  state_automatic_ = new StateAutomatic<T>(&this->data_, "automatic", node_handle);

  this->string2state.insert(std::pair<std::string, State<T> *>("passive", state_passive_));
  this->string2state.insert(std::pair<std::string, State<T> *>("raw", state_raw_));
  this->string2state.insert(std::pair<std::string, State<T> *>("automatic", state_automatic_));
  this->string2state.insert(std::pair<std::string, State<T> *>("attack_without_move", state_attack_without_move_));

  this->current_state_ = this->string2state["passive"];
}

template<typename T>
std::string FsmSentry<T>::getDesiredState() {
  this->control_mode_ = "rc";
  if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) {
    return "passive";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::MID) {
    return "raw";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
    return "automatic";
  } else {
    return "passive";
  }
}

template
class FsmSentry<double>;
template
class FsmSentry<float>;