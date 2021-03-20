//
// Created by bruce on 2021/1/17.
//

#include <rm_fsm/state_fly_slope.h>

template<typename T>
StateFlySlope<T>::StateFlySlope(FsmData<T> *fsm_data,
                                const std::string &state_string,
                                ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {

}

template<typename T>
void StateFlySlope<T>::onEnter() {
  ROS_INFO("Enter flyslope mode");
}

template<typename T>
void StateFlySlope<T>::run() {
  double linear_x, linear_y;
  double rate_yaw, rate_pitch;
  ros::Time now = ros::Time::now();

  this->loadParam();

  if (this->control_mode_ == "pc") { // pc control
    linear_x = this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s; // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;

    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;
  }

  this->setChassis(rm_msgs::ChassisCmd::FOLLOW, linear_x, linear_y, 0.0);
  this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0);
  this->setShoot(rm_msgs::ShootCmd::PASSIVE, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
}

template<typename T>
void StateFlySlope<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit flyslope mode");
}

template
class StateFlySlope<double>;
template
class StateFlySlope<float>;
