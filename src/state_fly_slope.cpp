//
// Created by bruce on 2021/1/17.
//

#include <rm_fsm/state_fly_slope.h>

template<typename T>
StateFlySlope<T>::StateFlySlope(FsmData<T> *fsm_data,
                                const std::string &state_string,
                                tf2_ros::TransformListener *tf_listener,
                                ros::NodeHandle &nh,
                                bool pc_control):State<T>(fsm_data, state_string, tf_listener, nh, pc_control) {

}

template<typename T>
void StateFlySlope<T>::onEnter() {
  ROS_INFO("[fsm] Enter flyslope mode");
}

template<typename T>
void StateFlySlope<T>::run() {
  double linear_x = 0, linear_y = 0, angular_z = 0;
  double rate_yaw = 0, rate_pitch = 0;
  int shoot_speed = 0;
  double shoot_hz = 0;
  ros::Time now = ros::Time::now();

  if (this->pc_control_) { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s) * 3.5; // W/S
    linear_y = -(this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d) * 3.5; // A/D
    angular_z = (this->data_->dbus_data_.key_q - this->data_->dbus_data_.key_e) * 6; // Q/E

    rate_yaw = -this->data_->dbus_data_.m_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.m_y * M_PI * 4;
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y * 3.5;
    linear_y = -this->data_->dbus_data_.ch_r_x * 3.5;
    angular_z = this->data_->dbus_data_.wheel * 6;

    rate_yaw = -this->data_->dbus_data_.ch_l_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.ch_l_y * M_PI * 4;
  }

  this->setChassis(this->data_->chassis_cmd_.FOLLOW, linear_x, linear_y, angular_z);
  this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch);
  this->setShoot(this->data_->shoot_cmd_.PASSIVE, shoot_speed, shoot_hz, now);
}

template<typename T>
void StateFlySlope<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("[fsm] Exit flyslope mode");
}

template
class StateFlySlope<double>;
template
class StateFlySlope<float>;
