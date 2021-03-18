//
// Created by bruce on 2020/12/16.
//

#include <rm_fsm/fsm_standard.h>

template<typename T>
FsmStandard<T>::FsmStandard(ros::NodeHandle &node_handle) : Fsm<T>(node_handle) {
  state_passive_ = new StatePassive<T>(&this->data_, "passive", node_handle);
  state_follow_ = new StateFollow<T>(&this->data_, "follow", node_handle);
  state_fly_slope_ = new StateFlySlope<T>(&this->data_, "flyslope", node_handle);
  state_burst_ = new StateBurst<T>(&this->data_, "burst", node_handle);

  this->string2state.insert(std::pair<std::string, State<T> *>("passive", state_passive_));
  this->string2state.insert(std::pair<std::string, State<T> *>("follow", state_follow_));
  this->string2state.insert(std::pair<std::string, State<T> *>("flyslope", state_fly_slope_));
  this->string2state.insert(std::pair<std::string, State<T> *>("burst", state_burst_));

  this->current_state_ = this->string2state["passive"];
}

template<typename T>
std::string FsmStandard<T>::getDesiredState() {

  if (this->control_mode_ == "pc") { // pc control
    if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_q) { // ctrl + q change state to passive
      return "passive";
    } else if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_w) { // ctrl + w change state to raw
      return "follow";
    } else if (this->data_.dbus_data_.key_ctrl
        && this->data_.dbus_data_.key_e) { // ctrl + e change state to fly slope
      return "flyslope";
    } else {
      return this->current_state_->state_name_;
    }
  } else if (this->control_mode_ == "rc") { // rc control
    if (this->data_.dbus_data_.s_r == this->data_.dbus_data_.DOWN) {
      return "passive";
    } else if (this->data_.dbus_data_.s_r == this->data_.dbus_data_.MID) {
      return "follow";
    } else if (this->data_.dbus_data_.s_r == this->data_.dbus_data_.UP) {
      return "flyslope";
    } else {
      return "passive";
    }
  } else {
    return "passive";
  }
}

template
class FsmStandard<double>;
template
class FsmStandard<float>;
