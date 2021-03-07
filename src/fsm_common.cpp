//
// Created by peter on 2020/12/3.
//

#include <rm_fsm/fsm_common.h>

/**
 * Constructor for the FSM State class.
 *
 * @param fsm_data holds all of the relevant control data
 * @param state_list_string2int the enumerated state name
 * @param state_string the string name of the current FSM state
 */
template<typename T>
State<T>::State(FsmData<T> *fsm_data, std::string state_name, ros::NodeHandle &nh, bool pc_control)
    : data_(fsm_data), state_name_(std::move(state_name)), state_nh_(nh), pc_control_(pc_control) {
  // load rc/pc control's parameter
  state_nh_ = ros::NodeHandle(nh, "remote_control");

  state_nh_.param("accel_x", accel_x_, 10.0);
  state_nh_.param("accel_y", accel_y_, 10.0);
  state_nh_.param("accel_angular", accel_angular_, 10.0);
  state_nh_.param("coefficient_x", coefficient_x_, 3.5);
  state_nh_.param("coefficient_y", coefficient_y_, 3.5);
  state_nh_.param("coefficient_angular", coefficient_angular_, 6.0);
  state_nh_.param("coefficient_yaw", coefficient_yaw_, 12.56);
  state_nh_.param("coefficient_pitch", coefficient_pitch_, 12.56);

  std::cout << "[FSM_State] Initialized FSM state: " << state_name_
            << std::endl;
}

/**
 *
 * @tparam T
 * @param chassis_mode
 * @param accel_x
 * @param accel_y
 * @param accel_z
 * @param linear_x
 * @param linear_y
 * @param angular_z
 */
template<typename T>
void State<T>::setChassis(uint8_t chassis_mode,
                          double linear_x,
                          double linear_y,
                          double angular_z) {
  this->data_->chassis_cmd_.mode = chassis_mode;

  this->data_->chassis_cmd_.accel.linear.x = accel_x_;
  this->data_->chassis_cmd_.accel.linear.y = accel_y_;
  this->data_->chassis_cmd_.accel.angular.z = accel_angular_;

  this->data_->cmd_vel.linear.x = linear_x * coefficient_x_;
  this->data_->cmd_vel.linear.y = linear_y * coefficient_y_;
  this->data_->cmd_vel.angular.z = angular_z * coefficient_angular_;

  this->data_->chassis_cmd_.effort_limit = 99;

  this->data_->vel_cmd_pub_.publish(this->data_->cmd_vel);
  this->data_->chassis_cmd_pub_.publish(this->data_->chassis_cmd_);
}

/**
 *
 * @tparam T
 * @param gimbal_mode
 * @param yaw_rate
 * @param pitch_rate
 */
template<typename T>
void State<T>::setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch) {
  this->data_->gimbal_cmd_.mode = gimbal_mode;

  this->data_->gimbal_cmd_.rate_yaw = rate_yaw * coefficient_yaw_;
  this->data_->gimbal_cmd_.rate_pitch = rate_pitch * coefficient_pitch_;

  this->data_->gimbal_cmd_pub_.publish(this->data_->gimbal_cmd_);
}

/**
 *
 * @tparam T
 * @param shoot_mode
 * @param shoot_num
 * @param now
 */
template<typename T>
void State<T>::setShoot(uint8_t shoot_mode, uint8_t shoot_speed, double shoot_hz, ros::Time now) {
  this->data_->shoot_cmd_.mode = shoot_mode;

  this->data_->shoot_cmd_.speed = shoot_speed;
  this->data_->shoot_cmd_.hz = shoot_hz;
  this->data_->shoot_cmd_.stamp = now;

  this->data_->shooter_cmd_pub_.publish(this->data_->shoot_cmd_);
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

  safety_checker_ = new SafetyChecker<T>(&data_);

  this->data_.init(nh_);

  string2state.insert(std::make_pair("invalid", nullptr));

  // Initialize a new FSM State with the control data
  current_state_ = string2state["invalid"];

  pc_control_ = getParam(node_handle, "pc_control", 0);

  // Enter the new current state cleanly
  ROS_INFO("[FSM] Current state is invalid.");

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
  // TODO: Safety check

  // run referee system and publish some referee data
  if (this->data_.referee_->flag) {
    data_.referee_->read();
  }

  // Run the robot control code if operating mode is not unsafe
  if (operating_mode_ != FsmOperatingMode::kEStop) {
    // Run normal controls if no transition is detected
    if (operating_mode_ == FsmOperatingMode::kNormal) {
      // Check the current state for any transition
      next_state_name_ = this->getDesiredState();

      // Detect a commanded transition
      if (next_state_name_ != current_state_->state_name_) {
        // Set the FSM operating mode to transitioning
        operating_mode_ = FsmOperatingMode::kTransitioning;

        // Get the next FSM State by name
        next_state_ = string2state[next_state_name_];
      } else {
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

      // Return the FSM to normal operation mode
      operating_mode_ = FsmOperatingMode::kNormal;
    } else {
      // Check the robot state for safe operation
      // TODO: Safety post check.
    }

  } else { // if ESTOP
    current_state_ = string2state["passive"];
    ROS_INFO("[FSM] Current state is passive.");
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