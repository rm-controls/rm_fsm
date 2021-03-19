//
// Created by bruce on 2021/1/17.
//

#include <rm_fsm/state_burst.h>

template<typename T>
StateBurst<T>::StateBurst(FsmData<T> *fsm_data,
                          const std::string &state_string,
                          ros::NodeHandle &nh):State<T>(fsm_data, state_string, nh) {
}

template<typename T>
void StateBurst<T>::onEnter() {
  ROS_INFO("Enter burst mode");
}

template<typename T>
void StateBurst<T>::run() {
  double linear_x, linear_y;
  double rate_yaw, rate_pitch;
  ros::Time now = ros::Time::now();

  if (this->control_mode_ == "pc") { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;

    if (this->data_->dbus_data_.p_r) {
      this->setGimbal(this->data_->gimbal_cmd_.TRACK, 0.0, 0.0, 8); // track sentry
    } else {
      this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch, 0);
    }

    if (this->data_->dbus_data_.p_l)
      this->setShoot(this->data_->shoot_cmd_.PUSH, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND,
                     this->shoot_hz_, now);
    else this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;

    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;

    if (this->data_->dbus_data_.s_r == this->data_->dbus_data_.UP) {
      this->setGimbal(this->data_->gimbal_cmd_.TRACK, 0.0, 0.0, 8); // track sentry
    } else {
      this->setGimbal(this->data_->gimbal_cmd_.RATE, rate_yaw, rate_pitch, 0);
    }

    if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.MID)
      this->setShoot(this->data_->shoot_cmd_.READY, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);
    else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.UP)
      this->setShoot(this->data_->shoot_cmd_.PUSH, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND,
                     this->shoot_hz_, now);
    else if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.DOWN)
      this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, 0.0, now);
  }

  this->setChassis(this->data_->chassis_cmd_.GYRO, linear_x, linear_y, 0.0);
}

template<typename T>
void StateBurst<T>::onExit() {
//Nothing to clean up when exiting
  ROS_INFO("Exit burst mode");
}

template
class StateBurst<double>;
template
class StateBurst<float>;

