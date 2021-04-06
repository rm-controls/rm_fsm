//
// Created by peter on 2021/3/15.
//

#include "rm_fsm/state_follow.h"

template<typename T>
StateFollow<T>::StateFollow(FsmData<T> *fsm_data,
                            const std::string &state_string,
                            ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StateFollow<T>::onEnter() {
  this->last_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
  this->last_shoot_mode_ = rm_msgs::ShootCmd::STOP;
  this->last_angular_z_ = 1;
  ROS_INFO("Enter follow mode");
}

template<typename T>
void StateFollow<T>::run() {
  double linear_x, linear_y, angular_z;
  double rate_yaw, rate_pitch;
  uint8_t chassis_mode, gimbal_mode, shoot_mode;
  uint8_t target_id;
  double bullet_speed;
  uint8_t shoot_speed;
  double shoot_hz = 0;
  ros::Time now = ros::Time::now();

  this->loadParam();

  if (this->control_mode_ == "pc") { // pc control
    // Check for press
    if (now - last_press_time_e_ < ros::Duration(0.2)) this->data_->dbus_data_.key_e = false;
    else last_press_time_e_ = now;
    if (now - last_press_time_q_ < ros::Duration(0.2)) this->data_->dbus_data_.key_q = false;
    else last_press_time_q_ = now;
    if (now - last_press_time_f_ < ros::Duration(0.2)) this->data_->dbus_data_.key_f = false;
    else last_press_time_f_ = now;

    // Send cmd to chassis
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D
    angular_z = 0;

    if (this->data_->dbus_data_.key_e && this->data_->dbus_data_.key_q) {
      chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
      is_spin_e_ = false;
      is_spin_q_ = false;
    } else if (this->data_->dbus_data_.key_e) {
      if (is_spin_e_) {
        chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
        is_spin_e_ = false;
      } else {
        chassis_mode = rm_msgs::ChassisCmd::GYRO;
        angular_z = 1;
        is_spin_e_ = true;
        is_spin_q_ = false;
      }
      this->last_chassis_mode_ = chassis_mode;
      this->last_angular_z_ = angular_z;
    } else if (this->data_->dbus_data_.key_q) {
      if (is_spin_q_) {
        chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
        is_spin_q_ = false;
      } else {
        chassis_mode = rm_msgs::ChassisCmd::GYRO;
        angular_z = -1;
        is_spin_q_ = true;
        is_spin_e_ = false;
      }
      this->last_chassis_mode_ = chassis_mode;
      this->last_angular_z_ = angular_z;
    } else {
      chassis_mode = this->last_chassis_mode_;
      angular_z = this->last_angular_z_;
    }
    this->setChassis(chassis_mode, linear_x, linear_y, angular_z);

    // Send cmd to gimbal
    rate_yaw = -this->data_->dbus_data_.m_x;
    rate_pitch = this->data_->dbus_data_.m_y;
    bullet_speed = 18;
    if (this->data_->dbus_data_.p_r) {
      this->data_->target_cost_function_->input(this->data_->track_data_array_);
      target_id = this->data_->target_cost_function_->output();
      if (target_id == 0) {
        gimbal_mode = rm_msgs::GimbalCmd::RATE;
      } else {
        gimbal_mode = rm_msgs::GimbalCmd::TRACK;
      }
    } else {
      gimbal_mode = rm_msgs::GimbalCmd::RATE;
    }
    this->setGimbal(gimbal_mode, rate_yaw, rate_pitch, target_id, bullet_speed);

    // Send cmd to shooter
    shoot_speed = rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
    shoot_mode = this->last_shoot_mode_;

    if (this->data_->dbus_data_.key_f) { // enable friction
      if (is_friction_ready_) {
        shoot_mode = rm_msgs::ShootCmd::STOP;
        is_friction_ready_ = false;
      } else {
        shoot_mode = rm_msgs::ShootCmd::READY;
        is_friction_ready_ = true;
      }
      this->last_shoot_mode_ = shoot_mode;
    } else {
      shoot_mode = this->last_shoot_mode_;
    }

    if (this->is_friction_ready_ && this->data_->dbus_data_.p_l) { // enable trigger
      this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
      shoot_hz = this->data_->shooter_heat_limit_->output();
      if (this->data_->dbus_data_.p_r) {
        if (now - this->data_->gimbal_des_error_.stamp > ros::Duration(0.5)) { // check time stamp
          this->data_->gimbal_des_error_.error_yaw = 0;
          this->data_->gimbal_des_error_.error_pitch = 0;
          ROS_WARN("The time stamp of gimbal track error is too old");
        }
        if (this->data_->gimbal_des_error_.error_yaw >= this->gimbal_error_limit_) { // check yaw error
          shoot_mode = rm_msgs::ShootCmd::READY;
          ROS_WARN("Gimbal track yaw error is too big, stop shooting");
        }
        if (this->data_->gimbal_des_error_.error_pitch >= this->gimbal_error_limit_) { // check pitch error
          shoot_mode = rm_msgs::ShootCmd::READY;
          ROS_WARN("Gimbal track pitch error is too big, stop shooting");
        }
      } else {
        shoot_mode = rm_msgs::ShootCmd::PUSH;
      }
    }
    this->setShoot(shoot_mode, shoot_speed, shoot_hz, now);

  } else { // rc control
    // Send command to chassis
    linear_x = this->data_->dbus_data_.ch_r_y;
    linear_y = -this->data_->dbus_data_.ch_r_x;
    if (this->data_->dbus_data_.wheel) { // enter gyro
      angular_z = this->data_->dbus_data_.wheel;
      chassis_mode = rm_msgs::ChassisCmd::GYRO;
    } else { // enter follow
      chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
    }
    this->setChassis(chassis_mode, linear_x, linear_y, angular_z);

    // Send command to gimbal
    rate_yaw = -this->data_->dbus_data_.ch_l_x;
    rate_pitch = -this->data_->dbus_data_.ch_l_y;
    shoot_speed = rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
    this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);

    // Send command to shooter
    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
      shoot_hz = this->data_->shooter_heat_limit_->output();
      shoot_mode = rm_msgs::ShootCmd::PUSH;
    } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
      shoot_mode = rm_msgs::ShootCmd::READY;
    } else {
      shoot_mode = rm_msgs::ShootCmd::STOP;
    }
    this->setShoot(shoot_mode, shoot_speed, shoot_hz, now);
  }
}
template<typename T>
void StateFollow<T>::onExit() {
  this->last_chassis_mode_ = rm_msgs::ChassisCmd::PASSIVE;
  this->last_shoot_mode_ = rm_msgs::ShootCmd::STOP;
  ROS_INFO("Exit follow mode");
}

template
class StateFollow<double>;
template
class StateFollow<float>;