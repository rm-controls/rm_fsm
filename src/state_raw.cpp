//
// Created by astro on 2020/12/8.
//

#include <rm_fsm/state_raw.h>

template<typename T>
StateRaw<T>::StateRaw(FsmData<T> *fsm_data,
                      const std::string &state_string,
                      ros::NodeHandle &nh):State<T>(fsm_data, state_string, nh) {
}

template<typename T>
void StateRaw<T>::onEnter() {
  ROS_INFO("Enter raw mode");
}

template<typename T>
void StateRaw<T>::run() {
  double linear_x, linear_y;
  double rate_yaw, rate_pitch;
  ros::Time now = ros::Time::now();

  // rc control
  linear_x = this->data_->dbus_data_.ch_r_y;
  linear_y = -this->data_->dbus_data_.ch_r_x;

  rate_yaw = -this->data_->dbus_data_.ch_l_x;
  rate_pitch = -this->data_->dbus_data_.ch_l_y;

  if (this->data_->dbus_data_.s_r == rm_msgs::DbusData::UP) {
    this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, 1);
  } else {
    this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0);
  }

  if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
    this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
    this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND,
                   this->data_->shooter_heat_limit_->output(), now);
  } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
    this->setShoot(rm_msgs::ShootCmd::READY, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
  } else {
    this->setShoot(rm_msgs::ShootCmd::PASSIVE, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
  }

  this->setChassis(rm_msgs::ChassisCmd::RAW, linear_x, linear_y, 0.0);
}
template<typename T>
void StateRaw<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit raw mode");
}

template
class StateRaw<double>;
template
class StateRaw<float>;
