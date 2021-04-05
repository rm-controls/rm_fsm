//
// Created by bruce on 2021/1/17.
//

#include "rm_fsm/state_burst.h"

template<typename T>
StateBurst<T>::StateBurst(FsmData<T> *fsm_data,
                          const std::string &state_string,
                          ros::NodeHandle &nh):State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StateBurst<T>::onEnter() {
  ROS_INFO("Enter burst mode");
}

template<typename T>
void StateBurst<T>::run() {
  double linear_x, linear_y;
  double rate_yaw, rate_pitch;
  uint8_t target_id;
  double bullet_speed = 7;
  ros::Time now = ros::Time::now();

  this->loadParam();

  if (this->control_mode_ == "pc") { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;

    if (this->data_->dbus_data_.p_r) {
      this->data_->target_cost_function_->input(this->data_->track_data_array_);
      target_id = this->data_->target_cost_function_->output();
      if (target_id == 0) {
        this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);
      } else {
        this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, target_id, bullet_speed);
      }
    } else {
      this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);
    }

    if (this->data_->dbus_data_.p_l) {
      this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, this->shoot_hz_, now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::STOP, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    }
  } else { // rc control
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;

    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;

    this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);

    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, this->shoot_hz_, now);
    } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
      this->setShoot(rm_msgs::ShootCmd::READY, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::STOP, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
    }
  }

  this->setChassis(rm_msgs::ChassisCmd::FOLLOW, linear_x, linear_y, 0.0);
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

