//
// Created by peter on 2021/3/15.
//

#include "rm_fsm/state_follow.h"

template<typename T>
StateFollow<T>::StateFollow(FsmData<T> *fsm_data,
                            const std::string &state_string,
                            ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {
  this->last_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
  this->last_shoot_mode_ = rm_msgs::ShootCmd::STOP;
}

template<typename T>
void StateFollow<T>::onEnter() {
  ROS_INFO("Enter follow mode");
}

template<typename T>
void StateFollow<T>::run() {
  double linear_x, linear_y, angular_z;
  double rate_yaw, rate_pitch;
  uint8_t chassis_mode, shoot_mode;
  uint8_t target_id;
  double bullet_speed = 7;
  uint8_t shoot_speed;
  double shoot_hz = 0;
  ros::Time now = ros::Time::now();

  this->loadParam();

  // Send cmd to chassis
  if (this->control_mode_ == "pc") { // pc control
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    chassis_mode = this->last_chassis_mode_;
    if (this->data_->dbus_data_.key_shift) {
      if (now - this->last_press_time_shift > ros::Duration(0.5)) { // check for press
        if (is_spin_) { // enter follow
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          this->is_spin_ = false;
        } else { // enter gyro
          chassis_mode = rm_msgs::ChassisCmd::GYRO;
          this->is_spin_ = true;
        }
        this->last_press_time_shift = now;
        this->last_chassis_mode_ = chassis_mode;
      }
    }

    this->setChassis(chassis_mode, linear_x, linear_y, 1.0);

    // Send cmd to gimbal
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

    shoot_speed = rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;

    shoot_mode = this->last_shoot_mode_;
    // Send cmd to shooter
    if (this->data_->dbus_data_.key_f) {
      if (now - this->last_press_time_f > ros::Duration(0.5)) {
        if (this->is_friction_ready_) {
          shoot_mode = rm_msgs::ShootCmd::STOP;
          this->is_friction_ready_ = false;
        } else {
          shoot_mode = rm_msgs::ShootCmd::READY;
          this->is_friction_ready_ = true;
        }
        this->last_press_time_f = now;
        this->last_shoot_mode_ = shoot_mode;
      }
    }

    if (this->is_friction_ready_) {
      if (this->data_->dbus_data_.p_l) {
        this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
        shoot_hz = this->data_->shooter_heat_limit_->output();
        shoot_mode = rm_msgs::ShootCmd::PUSH;
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
        }
      }
    }

    this->setShoot(shoot_mode, shoot_speed, shoot_hz, now);

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

    shoot_speed = rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;

    this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);

    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
      this->setShoot(rm_msgs::ShootCmd::PUSH, shoot_speed, 5, now);
    } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
      this->setShoot(rm_msgs::ShootCmd::READY, shoot_speed, 0.0, now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::STOP, shoot_speed, 0.0, now);
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