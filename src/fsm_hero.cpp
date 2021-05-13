//
// Created by bruce on 2021/1/29.
//

#include "rm_fsm/fsm_hero.h"

template<typename T>
FsmHero<T>::FsmHero(ros::NodeHandle &fsm_nh) : Fsm<T>(fsm_nh) {
  state_passive_ = new StatePassive<T>(&this->data_, "passive", fsm_nh);
  state_follow_ = new StateFollow<T>(&this->data_, "follow", fsm_nh);

  this->string2state.insert(std::pair<std::string, State<T> *>("passive", state_passive_));
  this->string2state.insert(std::pair<std::string, State<T> *>("follow", state_follow_));
  this->current_state_ = this->string2state["passive"];
}

template<typename T>
std::string FsmHero<T>::getDesiredState() {
  if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) { // pc control
    this->control_mode_ = "rc";
    enter_pc_ = false;
    return "passive";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::MID) { // follow mode
    this->control_mode_ = "rc";
    enter_pc_ = false;
    return "follow";
  } else if (this->data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
    this->control_mode_ = "pc";
    if (!enter_pc_) {
      enter_pc_ = true;
      return "passive";
    }
    if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_q) { // ctrl + q change state to passive
      return "passive";
    } else if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_w) { // ctrl + w change state to follow
      return "follow";
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

