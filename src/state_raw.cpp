//
// Created by astro on 2020/12/8.
//

#include "rm_fsm/state_raw.h"

template<typename T>
StateRaw<T>::StateRaw(FsmData<T> *fsm_data,
                      const std::string &state_string,
                      ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StateRaw<T>::onEnter() {
  this->actual_shoot_speed_ = this->safe_shoot_speed_;
  this->ultimate_shoot_speed_ = this->safe_shoot_speed_;
  ROS_INFO("Enter raw mode");
}

template<typename T>
void StateRaw<T>::run() {
  double linear_x, linear_y, angular_z;
  double rate_yaw, rate_pitch;
  uint8_t shoot_mode;
  double shoot_hz;
  ros::Time now = ros::Time::now();

  this->loadParam();

  // rc control
  // Send command to chassis
  linear_x = this->data_->dbus_data_.ch_r_y;
  linear_y = -this->data_->dbus_data_.ch_r_x;
  angular_z = this->data_->dbus_data_.wheel;
  this->setChassis(rm_msgs::ChassisCmd::RAW, linear_x, linear_y, angular_z);

  // Send command to gimbal
  rate_yaw = -this->data_->dbus_data_.ch_l_x;
  rate_pitch = -this->data_->dbus_data_.ch_l_y;
  this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);

  // Send command to shooter
  this->ultimate_shoot_speed_ = this->data_->referee_->getUltimateBulletSpeed(this->ultimate_shoot_speed_);
  shoot_hz = this->expect_shoot_hz_;
  if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
    shoot_mode = rm_msgs::ShootCmd::PUSH;
    this->data_->shooter_heat_limit_->input(this->data_->referee_, this->expect_shoot_hz_, this->safe_shoot_hz_);
    shoot_hz = this->data_->shooter_heat_limit_->output();
  } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
    shoot_mode = rm_msgs::ShootCmd::READY;
  } else {
    shoot_mode = rm_msgs::ShootCmd::STOP;
  }
  this->setShoot(shoot_mode, this->ultimate_shoot_speed_, shoot_hz, now);
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
