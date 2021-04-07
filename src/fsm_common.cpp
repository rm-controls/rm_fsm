//
// Created by peter on 2020/12/3.
//

#include <utility>
#include "rm_fsm/fsm_common.h"

template<typename T>
State<T>::State(ros::NodeHandle &nh, FsmData<T> *fsm_data, std::string state_name)
    : nh_(nh), data_(fsm_data), state_name_(std::move(state_name)) {

}

template<typename T>
void State<T>::loadParam() {
  if (control_mode_ == "pc") { // pc mode
    accel_x_ = getParam(nh_, "control_param/pc_param/accel_x", 10.0);
    accel_y_ = getParam(nh_, "control_param/pc_param/accel_y", 10.0);
    accel_angular_ = getParam(nh_, "control_param/pc_param/accel_angular", 10.0);
    brake_multiple_ = getParam(nh_, "control_param/pc_param/brake_multiple", 1);
    coefficient_x_ = getParam(nh_, "control_param/pc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/pc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/pc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/pc_param/coefficient_yaw", 125.6);
    coefficient_pitch_ = getParam(nh_, "control_param/pc_param/coefficient_pitch", 125.6);
    shoot_hz_ = getParam(nh_, "control_param/pc_param/shoot_hz", 5.0);
    gimbal_error_limit_ = getParam(nh_, "control_param/pc_param/gimbal_error_limit", 2.0);
    lowest_effort_ = getParam(nh_, "control_param/power_limit/lowest_effort_", 10);
  } else if (control_mode_ == "rc") { // rc mode
    accel_x_ = getParam(nh_, "control_param/rc_param/accel_x", 10.0);
    accel_y_ = getParam(nh_, "control_param/rc_param/accel_y", 10.0);
    accel_angular_ = getParam(nh_, "control_param/rc_param/accel_angular", 10.0);
    brake_multiple_ = getParam(nh_, "control_param/rc_param/brake_multiple", 1);
    coefficient_x_ = getParam(nh_, "control_param/rc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/rc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/rc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/rc_param/coefficient_yaw", 12.56);
    coefficient_pitch_ = getParam(nh_, "control_param/rc_param/coefficient_pitch", 12.56);
    shoot_hz_ = getParam(nh_, "control_param/rc_param/shoot_hz", 5.0);
    lowest_effort_ = getParam(nh_, "control_param/power_limit/lowest_effort_", 10);
  } else {
    ROS_ERROR("Cannot load control params.");
  }
}

template<typename T>
void State<T>::setChassis(uint8_t chassis_mode,
                          double linear_x,
                          double linear_y,
                          double angular_z) {
  double accel_x = accel_x_;
  double accel_y = accel_y_;
  double accel_angular = accel_angular_;

  data_->chassis_cmd_.mode = chassis_mode;

  if (linear_x == 0.0)
    accel_x = accel_x_ * brake_multiple_;

  if (linear_y == 0.0)
    accel_y = accel_y_ * brake_multiple_;

  if (angular_z == 0.0)
    accel_angular = accel_angular_ * brake_multiple_;

  data_->chassis_cmd_.accel.linear.x = accel_x;
  data_->chassis_cmd_.accel.linear.y = accel_y;
  data_->chassis_cmd_.accel.angular.z = accel_angular;

  if (data_->referee_->flag_) {
    if (data_->referee_->referee_data_.power_heat_data_.chassis_volt == 0) {
      data_->chassis_cmd_.effort_limit = lowest_effort_;
    } else {
      data_->power_limit_->input(data_->referee_->referee_data_,
                                 data_->referee_->power_manager_data_,
                                 true);
      data_->chassis_cmd_.effort_limit = data_->power_limit_->output();
    }
  } else {
    data_->chassis_cmd_.effort_limit = lowest_effort_;
  }

  data_->cmd_vel_.linear.x = linear_x * coefficient_x_;
  data_->cmd_vel_.linear.y = linear_y * coefficient_y_;
  data_->cmd_vel_.angular.z = angular_z * coefficient_angular_;

  data_->vel_cmd_pub_.publish(data_->cmd_vel_);
  data_->chassis_cmd_pub_.publish(data_->chassis_cmd_);
}

template<typename T>
void State<T>::setGimbal(uint8_t gimbal_mode,
                         double rate_yaw,
                         double rate_pitch,
                         uint8_t target_id,
                         double bullet_speed) {
  data_->gimbal_cmd_.mode = gimbal_mode;

  data_->gimbal_cmd_.rate_yaw = rate_yaw * coefficient_yaw_;
  data_->gimbal_cmd_.rate_pitch = rate_pitch * coefficient_pitch_;

  data_->gimbal_cmd_.target_id = target_id;
  data_->gimbal_cmd_.bullet_speed = bullet_speed;
  data_->gimbal_cmd_pub_.publish(data_->gimbal_cmd_);
}

template<typename T>
void State<T>::setShoot(uint8_t shoot_mode, uint8_t shoot_speed, double shoot_hz, ros::Time now) {
  data_->shoot_cmd_.mode = shoot_mode;

  data_->shoot_cmd_.speed = shoot_speed;
  data_->shoot_cmd_.hz = shoot_hz;
  data_->shoot_cmd_.stamp = now;

  data_->shooter_cmd_pub_.publish(data_->shoot_cmd_);
}

template<typename T>
void State<T>::setControlMode(const std::string &control_mode) {
  control_mode_ = control_mode;
}

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 * @tparam T
 * @param nh
 */
template<typename T>
Fsm<T>::Fsm(ros::NodeHandle &node_handle):nh_(node_handle) {
  tf_listener_ = new tf2_ros::TransformListener(tf_);

  data_.init(nh_);

  string2state.insert(std::make_pair("invalid", nullptr));
  // Initialize a new FSM State with the control data
  current_state_ = string2state["invalid"];

  // Initialize to not be in transition
  next_state_ = current_state_;

  // Initialize FSM mode to normal operation
  operating_mode_ = FsmOperatingMode::kNormal;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template<typename T>
void Fsm<T>::run() {
  uint8_t operate_type;
  // TODO: Safety check

  // run referee system
  data_.referee_->run();

  if (data_.referee_->flag_) {
    if (data_.referee_->count_ >= 15) { // 10hz
      data_.referee_->count_ = 0;
      if (data_.referee_->first_send_) {
        operate_type = kAdd;
        data_.referee_->first_send_ = false;
      } else {
        operate_type = kModify;
      }
      data_.referee_->drawCharacter(3, kYellow, operate_type, current_state_->state_name_);
    }
    data_.referee_->count_++;
  }

  // Run the robot control code if operating mode is not unsafe
  if (operating_mode_ != FsmOperatingMode::kEStop) {
    // Run normal controls if no transition is detected
    if (operating_mode_ == FsmOperatingMode::kNormal) {
      // Check the current state for any transition
      next_state_name_ = getDesiredState();

      // Detect a commanded transition
      if (next_state_name_ != current_state_->state_name_) {
        // Set the FSM operating mode to transitioning
        operating_mode_ = FsmOperatingMode::kTransitioning;

        // Get the next FSM State by name
        next_state_ = string2state[next_state_name_];
      } else {
        // Update control mode (pc/rc)
        current_state_->setControlMode(control_mode_);
        // Run the iteration for the current state normally
        current_state_->run();
      }
    }

    // Run the transition code while transition is occuring
    if (operating_mode_ == FsmOperatingMode::kTransitioning) {
      // Check the robot state for safe operation
      // TODO: Safety post check.

      // Exit the current state cleanly
      current_state_->onExit();
      // Complete the transition
      current_state_ = next_state_;
      // Enter the new current state cleanly
      current_state_->onEnter();
      // Update control mode (pc/rc)
      current_state_->setControlMode(control_mode_);

      // Return the FSM to normal operation mode
      operating_mode_ = FsmOperatingMode::kNormal;
    } else {
      // Check the robot state for safe operation
      // TODO: Safety post check.
    }

  } else { // if ESTOP
    current_state_ = string2state["passive"];
    ROS_INFO("Current state is passive.");
    next_state_name_ = current_state_->state_name_;
  }
}

// RobotRunner a template
template
class Fsm<float>;
template
class Fsm<double>;

template
class State<float>;
template
class State<double>;