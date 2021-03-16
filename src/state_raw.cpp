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

  if (this->control_mode_ == "pc") { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;

    if (this->data_->dbus_data_.p_l) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_->referee_data_, this->shoot_hz_);
      this->setShoot(this->data_->shoot_cmd_.PUSH, this->data_->shoot_cmd_.SPEED_16M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(), now);
    } else this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);

  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;

    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;

    if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.MID)
      this->setShoot(this->data_->shoot_cmd_.READY, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);
    else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.UP) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_->referee_data_, this->shoot_hz_);
      this->setShoot(this->data_->shoot_cmd_.PUSH, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(), now);
    } else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.DOWN) {
      this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);
    }
  }

  this->setChassis(this->data_->chassis_cmd_.RAW, linear_x, linear_y, 0.0);
  this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch);
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
