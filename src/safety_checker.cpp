//
// Created by luohx on 7/20/20.
//

#include <rm_fsm/safety_checker.h>
#include <math_utilities.h>

template<typename T>
SafetyChecker<T>::SafetyChecker(FsmData<T> *dataIn):
    data_(dataIn) {
  last_dubs_stamp_ = ros::Time::now();
}

template<typename T>
bool SafetyChecker<T>::checkPower() {
  return false;
}

template<typename T>
bool SafetyChecker<T>::checkPos() {
  geometry_msgs::Vector3 pos = this->data_->euler_;
  bool pos_fine;
  if (fabs(pos.x) >= M_PI_2 || fabs(pos.y) >= M_PI_2) {
    pos_fine = false;
    ROS_ERROR("[fsm] Safety: Rollover!");
  } else
    pos_fine = true;
  return pos_fine;
}

template<typename T>
bool SafetyChecker<T>::checkTemper() {
  bool temper_fine = true;
  for (const auto &item:this->data_->joint_data_.temper) {
    if (item <= 100) {
      temper_fine = true;
    } else {
      ROS_ERROR("[fsm] Safety: Motor too hot!");
      temper_fine = false;
    }
  }
  return temper_fine;
}

template<typename T>
bool SafetyChecker<T>::checkDbusUpdate() {
  rm_msgs::DbusData dbus_data = this->data_->dbus_data_;
  bool dbus_fine;
  if (dbus_data.stamp.toSec() - last_dubs_stamp_.toSec() <= 0.1) {
    last_dubs_stamp_ = dbus_data.stamp;
    dbus_fine = true;
  } else {
    ROS_ERROR("[fsm] Safety: Dbus data update time out!");
    last_dubs_stamp_ = dbus_data.stamp;
    dbus_fine = false;
  }
  return dbus_fine;
}

template
class SafetyChecker<float>;
template
class SafetyChecker<double>;



