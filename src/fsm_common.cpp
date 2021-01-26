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
State<T>::State(FsmData<T> *fsm_data,
                std::string state_name,
                ros::NodeHandle &nh,
                bool pc_control)
    : data_(fsm_data),
      state_name_(std::move(state_name)),
      state_nh_(nh),
      pc_control_(pc_control) {
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

  this->data_->chassis_cmd_.accel.linear.x = 10;
  this->data_->chassis_cmd_.accel.linear.y = 10;
  this->data_->chassis_cmd_.accel.angular.z = 10;

  this->data_->cmd_vel.linear.x = linear_x;
  this->data_->cmd_vel.linear.y = linear_y;
  this->data_->cmd_vel.angular.z = angular_z;
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

  this->data_->gimbal_cmd_.rate_yaw = rate_yaw;
  this->data_->gimbal_cmd_.rate_pitch = rate_pitch;

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

  // Initialize the FSM with the Passive FSM State
  init();
}

/**
 * Initialize the Control FSM with the default settings. Should be set to
 * Passive state and Normal operation mode.
 * @attention Passive state is replaced by invalid state temporarily.
 */
template<typename T>
void Fsm<T>::init() {
  this->data_.init(nh_);

  string2state.insert(std::make_pair("invalid", nullptr));

  // Initialize a new FSM State with the control data
  current_state_ = string2state["invalid"];
  pc_control_ = 0;
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

    data_.referee_->referee_pub_data_.chassis_volt = data_.referee_->referee_data_.power_heat_data_.chassis_volt;
    data_.referee_->referee_pub_data_.chassis_current = data_.referee_->referee_data_.power_heat_data_.chassis_current;
    data_.referee_->referee_pub_data_.chassis_power = data_.referee_->referee_data_.power_heat_data_.chassis_power;
    data_.referee_->referee_pub_data_.chassis_power_buffer =
        data_.referee_->referee_data_.power_heat_data_.chassis_power_buffer;

    data_.referee_pub_.publish(data_.referee_->referee_pub_data_);
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

        // Print transition initialized info
        //PrintInfo(1);

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

      // Print finalizing transition info
      //PrintInfo(2);

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

  // Print the current state of the FSM
  printInfo(0);

  // Increase the iteration counter
  iter_++;
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template<typename T>
void Fsm<T>::printInfo(int opt) {
  switch (opt) {
    case 0:  // Normal printing case at regular intervals
      // Increment printing iteration
      print_iter_++;

      // Print at commanded frequency
      if (print_iter_ == print_num_) {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
            << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << iter_ << "\n";
        if (operating_mode_ == FsmOperatingMode::kNormal) {
          std::cout << "Operating Mode: NORMAL in " << current_state_->state_name_
                    << "\n";

        } else if (operating_mode_ == FsmOperatingMode::kTransitioning) {
          std::cout << "Operating Mode: TRANSITIONING from "
                    << current_state_->state_name_ << " to "
                    << next_state_->state_name_ << "\n";

        } else if (operating_mode_ == FsmOperatingMode::kEStop) {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << std::endl;

        // Reset iteration counter
        print_iter_ = 0;
      }

      // Print robot info about the robot's status
      // data._gaitScheduler->printGaitInfo();
      // data._desiredStateCommand->printStateCommandInfo();

      break;

    case 1:  // Initializing FSM State transition
      std::cout << "[CONTROL FSM] Transition initialized from "
                << current_state_->state_name_ << " to " << next_state_->state_name_
                << "\n"
                << std::endl;

      break;

    case 2:  // Finalizing FSM State transition
      std::cout << "[CONTROL FSM] Transition finalizing from "
                << current_state_->state_name_ << " to " << next_state_->state_name_
                << "\n"
                << std::endl;

      break;
    default:break;
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