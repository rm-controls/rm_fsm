//
// Created by bruce on 2021/1/17.
//

#include <rm_fsm/state_burst.h>

template<typename T>
StateBurst<T>::StateBurst(FsmData<T> *fsm_data,
                          const std::string &state_string,
                          tf2_ros::TransformListener *tf_listener,
                          ros::NodeHandle &nh,
                          bool pc_control):State<T>(fsm_data, state_string, tf_listener, nh, pc_control) {

}

template<typename T>
void StateBurst<T>::onEnter() {
  ROS_INFO("[fsm] Enter burst mode");
}

template<typename T>
void StateBurst<T>::run() {
  double linear_x = 0, linear_y = 0, angular_z = 0;
  double rate_yaw = 0, rate_pitch = 0;
  double shoot_num = 0;
  ros::Time now = ros::Time::now();

  if (this->pc_control_) { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s) * 3.5; // W/S
    linear_y = -(this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d) * 3.5; // A/D
    angular_z = (this->data_->dbus_data_.key_q - this->data_->dbus_data_.key_e) * 6; // Q/E

    rate_yaw = -this->data_->dbus_data_.m_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.m_y * M_PI * 4;

    shoot_num = this->data_->dbus_data_.p_l;
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y * 3.5;
    linear_y = -this->data_->dbus_data_.ch_r_x * 3.5;
    angular_z = this->data_->dbus_data_.wheel * 6;

    rate_yaw = -this->data_->dbus_data_.ch_l_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.ch_l_y * M_PI * 4;

    if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.MID) shoot_num = 1;
    else shoot_num = 0;
  }

  this->setChassis(this->data_->chassis_cmd_.GYRO, linear_x, linear_y, angular_z);
  this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch);
  this->setShoot(this->data_->shoot_cmd_.READY, shoot_num, now);
}

template<typename T>
void StateBurst<T>::onExit() {
//Nothing to clean up when exiting
  ROS_INFO("[fsm] Exit burst mode");
}

template
class StateBurst<double>;
template
class StateBurst<float>;

