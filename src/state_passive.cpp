//
// Created by astro on 2020/12/8.
//

#include "rm_fsm/state_passive.h"

template<typename T>
StatePassive<T>::StatePassive(FsmData<T> *fsm_data,
                              const std::string &state_string,
                              ros::NodeHandle &nh):
    State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StatePassive<T>::onEnter() {
  ROS_INFO("Enter passive mode");
}

template<typename T>
void StatePassive<T>::run() {
  uint8_t graph_operate_type;
  ros::Time now = ros::Time::now();

  this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
  this->setGimbal(rm_msgs::GimbalCmd::PASSIVE, 0.0, 0.0, 0, 0.0);
  this->setShoot(rm_msgs::ShootCmd::STOP, 0, 0.0, now);

  // Refresh client graph
  if (this->data_->dbus_data_.key_x) {
    graph_operate_type = kAdd;
  } else {
    graph_operate_type = kUpdate;
  }

  this->data_->referee_->write(this->state_name_, graph_operate_type, false, false);
}

template<typename T>
void StatePassive<T>::onExit() {
//Nothing to clean up when exiting
  ROS_INFO("Exit passive mode");
}

template
class StatePassive<double>;
template
class StatePassive<float>;