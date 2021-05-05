//
// Created by peter on 2021/3/15.
//

#include "rm_fsm/state_follow.h"

template<typename T>
StateFollow<T>::StateFollow(FsmData<T> *fsm_data,
                            const std::string &state_string,
                            ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {
  normal_critical_speed_ = getParam(nh, "control_param/pc_param/normal_critical_speed", 1);
  burst_critical_speed_ = getParam(nh, "control_param/pc_param/burst_critical_speed", 2);
  normal_angular_ = getParam(nh, "control_param/pc_param/normal_angular", 1);
  burst_angular_ = getParam(nh, "control_param/pc_param/burst_angular", 2);
  spin_sin_amplitude_ = getParam(nh, "control_param/pc_param/spin_sin_amplitude", 1.0);
  spin_sin_frequency_ = getParam(nh, "control_param/pc_param/spin_sin_frequency", 6.28);
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
  double shoot_hz = 0;
  ros::Time now = ros::Time::now();
  double normal_critical_speed, burst_critical_speed, normal_angular, burst_angular;

  this->loadParam();
  this->actual_shoot_speed_ = this->safe_shoot_speed_;
  this->ultimate_shoot_speed_ = this->safe_shoot_speed_;

  if (this->control_mode_ == "pc") { // pc control
    // Check for press
    if (this->data_->dbus_data_.key_g) {
      if (now - last_press_time_g_ < ros::Duration(0.5)) this->data_->dbus_data_.key_g = false;
      else last_press_time_g_ = now;
    }
    if (this->data_->dbus_data_.key_r) {
      if (now - last_press_time_r_ < ros::Duration(0.5)) this->data_->dbus_data_.key_r = false;
      else last_press_time_r_ = now;
    }
    if (this->data_->dbus_data_.key_q) {
      if (now - last_press_time_q_ < ros::Duration(0.5)) this->data_->dbus_data_.key_q = false;
      else last_press_time_q_ = now;
    }
    if (this->data_->dbus_data_.key_c) {
      if (now - last_press_time_c_ < ros::Duration(0.5)) this->data_->dbus_data_.key_c = false;
      else last_press_time_c_ = now;
    }
    // Send cmd to chassis
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D

    // Update by robot level
    normal_critical_speed =
        normal_critical_speed_ * pow(1.1, this->data_->referee_->referee_data_.game_robot_status_.robot_level);
    burst_critical_speed =
        burst_critical_speed_ * pow(1.1, this->data_->referee_->referee_data_.game_robot_status_.robot_level);
    normal_angular =
        normal_angular_ * pow(1.1, this->data_->referee_->referee_data_.game_robot_status_.robot_level);
    burst_angular =
        burst_angular_ * pow(1.1, this->data_->referee_->referee_data_.game_robot_status_.robot_level);
    //switch attack mode
    if (this->data_->dbus_data_.key_c) {
      if (only_attack_base_) {
        only_attack_base_ = false;
      } else {
        only_attack_base_ = true;
      }
    }
    // Switch spin mode
    if (this->data_->dbus_data_.key_g) {
      if (is_spin_) {
        is_spin_ = false;
        this->last_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
      } else {
        is_spin_ = true;
        twist_ = false;
      }
    }
    // Switch spin mode
    if (this->data_->dbus_data_.key_r) {
      if (twist_) {
        twist_ = false;
        this->last_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
      } else {
        twist_ = true;
        is_spin_ = false;
      }
    }
    if (is_spin_) {
      if (this->data_->dbus_data_.key_shift) { // burst mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.y, 2)
            <= pow(burst_critical_speed, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::GYRO;
          angular_z = burst_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = burst_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        }
      } else { // normal mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.y, 2)
            <= pow(normal_critical_speed, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::GYRO;
          angular_z = normal_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = normal_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        }
      }
      this->last_chassis_mode_ = chassis_mode;
      this->last_angular_z_ = angular_z;
    } else if (twist_) {
      if (this->data_->dbus_data_.key_shift) { // burst mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.y, 2)
            <= pow(burst_critical_speed, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::TWIST;
          angular_z = burst_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = burst_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        }
      } else { // normal mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.y, 2)
            <= pow(normal_critical_speed, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::TWIST;
          angular_z = normal_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = normal_angular + spin_sin_amplitude_ * abs(sin(spin_sin_frequency_ * now.toNSec()));
        }
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
    this->actual_shoot_speed_ = this->data_->referee_->getActualBulletSpeed(this->actual_shoot_speed_);
    // Switch track mode
    if (this->data_->dbus_data_.p_r) {
      this->data_->target_cost_function_->input(this->data_->track_data_array_,
                                                this->data_->referee_->referee_data_.game_robot_hp_,
                                                only_attack_base_);
      target_id = this->data_->target_cost_function_->output();
      if (target_id == 0) {
        gimbal_mode = rm_msgs::GimbalCmd::RATE;
      } else {
        gimbal_mode = rm_msgs::GimbalCmd::TRACK;
      }
    } else {
      gimbal_mode = rm_msgs::GimbalCmd::RATE;
    }
    this->setGimbal(gimbal_mode, rate_yaw, rate_pitch, target_id, this->actual_shoot_speed_);

    // Send cmd to shooter
    this->ultimate_shoot_speed_ = this->data_->referee_->getUltimateBulletSpeed(this->ultimate_shoot_speed_);
    // Switch friction mode
    if (this->data_->dbus_data_.key_f) {
      if (is_friction_ready_) {
        is_friction_ready_ = false;
      }
      shoot_mode = rm_msgs::ShootCmd::STOP;
      this->last_shoot_mode_ = shoot_mode;
    } else {
      shoot_mode = this->last_shoot_mode_;
    }

    // Switch shooter heat limit mode
    if (this->data_->dbus_data_.key_q) {
      is_burst_ = !is_burst_;
    }

    if (this->data_->dbus_data_.p_l) { // enable trigger
      shoot_mode = rm_msgs::ShootCmd::READY;
      this->last_shoot_mode_ = shoot_mode;
      is_friction_ready_ = true;
      if (is_burst_) { // ignore shooter heat limit
        shoot_hz = this->expect_shoot_hz_;
      } else {
        this->data_->shooter_heat_limit_->input(this->data_->referee_, this->expect_shoot_hz_, this->safe_shoot_hz_);
        shoot_hz = this->data_->shooter_heat_limit_->output();
      }

      if (this->data_->dbus_data_.p_r) {
        if (now - this->data_->gimbal_des_error_.stamp > ros::Duration(1.0)) { // check time stamp
          this->data_->gimbal_des_error_.error = 0;
          ROS_WARN("The time stamp of gimbal track error is too old");
        }
        if (std::abs(this->data_->gimbal_des_error_.error) >= this->gimbal_error_limit_) { // check  error
          shoot_mode = rm_msgs::ShootCmd::READY;
          ROS_WARN("Gimbal track error is too big, stop shooting");
        } else {
          shoot_mode = rm_msgs::ShootCmd::PUSH;
        }
      } else {
        shoot_mode = rm_msgs::ShootCmd::PUSH;
      }
    }
    this->setShoot(shoot_mode, this->ultimate_shoot_speed_, shoot_hz, now);

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

    // Send command to shooter
    this->ultimate_shoot_speed_ = this->data_->referee_->getUltimateBulletSpeed(this->ultimate_shoot_speed_);
    this->data_->target_cost_function_->input(this->data_->track_data_array_,
                                              this->data_->referee_->referee_data_.game_robot_hp_,
                                              only_attack_base_);
    target_id = this->data_->target_cost_function_->output();
    if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::UP) {
      if (target_id == 0) {
        if (last_target_id_ != 0)
          this->setGimbal(rm_msgs::GimbalCmd::TRACK, rate_yaw, rate_pitch, last_target_id_, this->actual_shoot_speed_);
        else
          this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);
      } else {
        last_target_id_ = target_id;
        this->setGimbal(rm_msgs::GimbalCmd::TRACK, rate_yaw, rate_pitch, target_id, this->actual_shoot_speed_);
      }
      this->data_->shooter_heat_limit_->input(this->data_->referee_, this->expect_shoot_hz_, this->safe_shoot_hz_);
      shoot_hz = this->data_->shooter_heat_limit_->output();
      shoot_mode = rm_msgs::ShootCmd::PUSH;
    } else if (this->data_->dbus_data_.s_l == rm_msgs::DbusData::MID) {
      if (target_id == 0) {
        if (last_target_id_ != 0)
          this->setGimbal(rm_msgs::GimbalCmd::TRACK, rate_yaw, rate_pitch, last_target_id_, this->actual_shoot_speed_);
        else
          this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);
      } else {
        last_target_id_ = target_id;
        this->setGimbal(rm_msgs::GimbalCmd::TRACK, rate_yaw, rate_pitch, target_id, this->actual_shoot_speed_);
      }
      shoot_mode = rm_msgs::ShootCmd::READY;
    } else {
      shoot_mode = rm_msgs::ShootCmd::STOP;
      this->setGimbal(rm_msgs::GimbalCmd::RATE, rate_yaw, rate_pitch, 0, 0.0);
    }

    this->setShoot(shoot_mode, this->ultimate_shoot_speed_, shoot_hz, now);
  }
}
template<typename T>
void StateFollow<T>::onExit() {
  this->last_chassis_mode_ = rm_msgs::ChassisCmd::PASSIVE;
  this->last_shoot_mode_ = rm_msgs::ShootCmd::STOP;
  this->graph_operate_type_ = kUpdate;
  ROS_INFO("Exit follow mode");
}

template
class StateFollow<double>;
template
class StateFollow<float>;