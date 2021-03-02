//
// Created by astro on 2020/12/8.
//

#include <rm_fsm/state_raw.h>

template<typename T>
StateRaw<T>::StateRaw(FsmData<T> *fsm_data,
                      const std::string &state_string,
                      ros::NodeHandle &nh,
                      bool pc_control):State<T>(fsm_data, state_string, nh, pc_control) {
  shoot_hz = getParam(this->state_nh_, "shoot_hz", 5);
}

template<typename T>
void StateRaw<T>::onEnter() {
  ROS_INFO("[fsm] Enter raw mode");
}

template<typename T>
void StateRaw<T>::run() {
  double linear_x = 0, linear_y = 0, angular_z = 0;
  double rate_yaw = 0, rate_pitch = 0;
  ros::Time now = ros::Time::now();

  if (this->pc_control_) { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s) * 3.5; // W/S
    linear_y = -(this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d) * 3.5; // A/D
    angular_z = (this->data_->dbus_data_.key_q - this->data_->dbus_data_.key_e) * 6; // Q/E

    rate_yaw = -this->data_->dbus_data_.m_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.m_y * M_PI * 4;

    if (this->data_->dbus_data_.p_l)
      this->setShoot(this->data_->shoot_cmd_.PUSH,
                     this->data_->shoot_cmd_.SPEED_16M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(),
                     now);
    else this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, shoot_hz, now);
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y * 3.5;
    linear_y = -this->data_->dbus_data_.ch_r_x * 3.5;
    angular_z = this->data_->dbus_data_.wheel * 6;

    rate_yaw = -this->data_->dbus_data_.ch_l_x * M_PI * 4;
    rate_pitch = -this->data_->dbus_data_.ch_l_y * M_PI * 4;

    if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.MID)
      this->setShoot(this->data_->shoot_cmd_.READY,
                     this->data_->shoot_cmd_.SPEED_10M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(),
                     now);
    else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.UP) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_->referee_data_);
      this->setShoot(this->data_->shoot_cmd_.PUSH,
                     this->data_->shoot_cmd_.SPEED_10M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(),
                     now);
    } else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.DOWN) {
      this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, shoot_hz, now);
    }

    this->setChassis(this->data_->chassis_cmd_.GYRO, linear_x, linear_y, angular_z);
    this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch);
  }
}
template<typename T>
void StateRaw<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("[fsm] Exit raw mode");
}

template
class StateRaw<double>;
template
class StateRaw<float>;
