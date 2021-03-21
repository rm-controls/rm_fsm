//
// Created by bruce on 2021/1/29.
//

#include "rm_fsm/fsm_hero.h"

template<typename T>
FsmHero<T>::FsmHero(ros::NodeHandle &node_handle) : Fsm<T>(node_handle) {
  state_passive_ = new StatePassive<T>(&this->data_, "passive", node_handle);
  state_follow_ = new StateFollow<T>(&this->data_, "follow", node_handle);

  this->string2state.insert(std::pair<std::string, State<T> *>("passive", state_passive_));
  this->string2state.insert(std::pair<std::string, State<T> *>("follow", state_follow_));
  this->current_state_ = this->string2state["passive"];
}

template<typename T>
std::string FsmHero<T>::getDesiredState() {
  if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) { // pc control
    this->control_mode_ = "rc";
    return "passive";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::MID) { // follow mode
    this->control_mode_ = "rc";
    return "follow";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
    this->control_mode_ = "pc";
    if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_q) { // ctrl + q change state to passive
      return "passive";
    } else if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_w) { // ctrl + w change state to follow
      return "follow";
    } else if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_e) { // ctrl + e change state to fly slope
      return "flyslope";
    } else {
      return this->current_state_->state_name_;
    }
  } else {
    this->control_mode_ = "rc";
    return "passive";
  }
}

template
class FsmHero<double>;
template
class FsmHero<float>;

