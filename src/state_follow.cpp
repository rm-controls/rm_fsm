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
  robot_type_ = getParam(nh, "robot_type", (std::string) "hero");
}

template<typename T>
void StateFollow<T>::onEnter() {
  this->last_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
  this->last_shoot_mode_ = rm_msgs::ShootCmd::STOP;
  this->last_angular_z_ = 1;
  this->graph_operate_type_ = kAdd;
  ROS_INFO("Enter follow mode");
}

template<typename T>
void StateFollow<T>::run() {
  double linear_x, linear_y, angular_z;
  double rate_yaw, rate_pitch;
  uint8_t chassis_mode, gimbal_mode, shoot_mode;
  uint8_t target_id;
  uint8_t shoot_speed;
  double shoot_hz = 0;
  ros::Time now = ros::Time::now();
  uint8_t graph_operate_type;

  this->loadParam();

  if (this->control_mode_ == "pc") { // pc control
    // Check for press
    if (this->data_->dbus_data_.key_e) {
      if (now - last_press_time_e_ < ros::Duration(0.25)) this->data_->dbus_data_.key_e = false;
      else last_press_time_e_ = now;
    }
    if (this->data_->dbus_data_.key_f) {
      if (now - last_press_time_f_ < ros::Duration(0.25)) this->data_->dbus_data_.key_f = false;
      else last_press_time_f_ = now;
    }
    if (this->data_->dbus_data_.key_q) {
      if (now - last_press_time_q_ < ros::Duration(0.25)) this->data_->dbus_data_.key_q = false;
      else last_press_time_q_ = now;
    }
    if (robot_type_ != "standard") {
      this->data_->dbus_data_.key_shift = false;
    }

    // Send cmd to chassis
    linear_x = (this->data_->dbus_data_.key_w - this->data_->dbus_data_.key_s); // W/S
    linear_y = (this->data_->dbus_data_.key_a - this->data_->dbus_data_.key_d); // A/D
    angular_z = 0;
    // Switch spin mode
    if (this->data_->dbus_data_.key_e) {
      is_spin_ = !is_spin_;
    }
    if (is_spin_) {
      if (this->data_->dbus_data_.key_shift) { // burst mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.x, 2)
            <= pow(normal_critical_speed_, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::GYRO;
          angular_z = normal_angular_;
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = normal_angular_;
        }
      } else { // normal mode
        if (pow(this->data_->odom_.twist.twist.linear.x, 2) + pow(this->data_->odom_.twist.twist.linear.x, 2)
            <= pow(burst_critical_speed_, 2)) {
          chassis_mode = rm_msgs::ChassisCmd::GYRO;
          angular_z = burst_angular_;
        } else {
          chassis_mode = rm_msgs::ChassisCmd::FOLLOW;
          angular_z = normal_angular_;
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
    // Switch track mode
    if (this->data_->dbus_data_.p_r) {
      this->data_->target_cost_function_->input(this->data_->track_data_array_);
      target_id = this->data_->target_cost_function_->output();
      if (target_id == 0) {
        gimbal_mode = rm_msgs::GimbalCmd::RATE;
        shoot_speed = this->shoot_speed_;
      } else {
        gimbal_mode = rm_msgs::GimbalCmd::TRACK;
        shoot_speed = this->data_->referee_->getBulletSpeed(this->shoot_speed_);
      }
    } else {
      gimbal_mode = rm_msgs::GimbalCmd::RATE;
    }
    this->setGimbal(gimbal_mode, rate_yaw, rate_pitch, target_id, shoot_speed);

    // Send cmd to shooter
    // Switch friction mode
    if (this->data_->dbus_data_.key_f) {
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

    // Switch shooter heat limit mode
    if (this->data_->dbus_data_.key_q) {
      is_burst_ = !is_burst_;
    }

    if (this->is_friction_ready_ && this->data_->dbus_data_.p_l) { // enable trigger
      if (is_burst_) { // ignore shooter heat limit
        shoot_hz = this->shoot_hz_;
      } else {
        this->data_->shooter_heat_limit_->input(this->data_->referee_, this->shoot_hz_);
        shoot_hz = this->data_->shooter_heat_limit_->output();
      }

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

    // Send command to sentry
    if (this->data_->dbus_data_.key_z) {
      int receiver_id;
      std::vector<uint8_t> sentry_cmd(1, 1);
      if (this->data_->referee_->robot_id_ > 100) receiver_id = kBlueSentry;
      else receiver_id = kRedSentry;
      this->data_->referee_->sendInteractiveData(0x0200, receiver_id, sentry_cmd);
    }

    // Refresh client graph
    if (this->data_->dbus_data_.key_x) {
      graph_operate_type = kAdd;
    } else {
      graph_operate_type = kUpdate;
    }

    this->data_->referee_->write(this->state_name_, graph_operate_type, is_burst_, this->data_->dbus_data_.key_shift);

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
    shoot_speed = this->shoot_speed_;
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
  this->graph_operate_type_ = kUpdate;
  ROS_INFO("Exit follow mode");
}

template
class StateFollow<double>;
template
class StateFollow<float>;