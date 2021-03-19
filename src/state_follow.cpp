//
// Created by peter on 2021/3/15.
//

#include <rm_fsm/state_follow.h>

template<typename T>
StateFollow<T>::StateFollow(FsmData<T> *fsm_data,
                            const std::string &state_string,
                            ros::NodeHandle &nh):State<T>(fsm_data, state_string, nh) {
}

template<typename T>
void StateFollow<T>::onEnter() {
  ROS_INFO("Enter follow mode");
}

template<typename T>
void StateFollow<T>::run() {
  double linear_x, linear_y, angular_z;
  double rate_yaw, rate_pitch;
  ros::Time now = ros::Time::now();

  if (this->control_mode_ == "pc") { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D
    if (this->data_->dbus_data_.key_shift) {
      if (is_spin) { // enter follow
        this->setChassis(rm_msgs::ChassisCmd::FOLLOW, linear_x, linear_y, 0.0);
      } else { // enter gyro
        this->setChassis(rm_msgs::ChassisCmd::GYRO, linear_x, linear_y, 1.0);
      }
    }

    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;

    if (this->data_->dbus_data_.p_r) {
      this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, 8);
    } else {
      this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0);
    }

    if (this->data_->dbus_data_.p_l) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_->referee_data_, this->shoot_hz_);
      this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_16M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(), now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::PASSIVE, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    }

  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;

    if (this->data_->dbus_data_.wheel) { // enter gyro
      angular_z = this->data_->dbus_data_.wheel;
      this->setChassis(rm_msgs::ChassisCmd::GYRO, linear_x, linear_y, angular_z);
    } else { // enter follow
      this->setChassis(rm_msgs::ChassisCmd::FOLLOW, linear_x, linear_y, 0.0);
    }

    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;

    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, 8);
    } else {
      this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0);
    }

    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_->referee_data_, this->shoot_hz_);
      this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND,
                     this->data_->shooter_heat_limit_->output(), now);
    } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
      this->setShoot(rm_msgs::ShootCmd::READY, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::PASSIVE, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    }
  }
}
template<typename T>
void StateFollow<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit follow mode");
}

template
class StateFollow<double>;
template
class StateFollow<float>;