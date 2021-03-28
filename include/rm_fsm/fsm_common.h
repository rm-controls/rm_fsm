//
// Created by peter on 2020/12/3.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_

#include <iostream>
#include <tf/transform_listener.h>
#include <queue>
#include <rm_common/ros_utilities.h>
#include "rm_common/ori_tool.h"
#include <utility>
#include <control_toolbox/pid.h>
#include "rm_fsm/fsm_data.h"

/**
 * A base fsm state class for all robots.
 * @tparam T
 */
template<typename T>
class State {
 public:
  // Generic constructor fo all states
  State(ros::NodeHandle &nh, FsmData<T> *fsm_data, std::string state_string);

  ros::NodeHandle nh_;

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;

  // Run the normal behavior for the state
  virtual void run() = 0;

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0;

  // Load params from yaml file
  void loadParam();

  // Base controllers.
  void setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z);
  void setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch, uint8_t target_id);
  void setShoot(uint8_t shoot_mode, uint8_t shoot_speed, double shoot_hz, ros::Time now);

  void setControlMode(const std::string &control_mode);

  // Holds all of the relevant control data
  FsmData<T> *data_;

  // FSM State info
  std::string state_name_;     // enumerated name of the current state

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  std::string control_mode_;

  // chassis fsm control accelerate
  double accel_x_ = 0.0;
  double accel_y_ = 0.0;
  double accel_angular_ = 0.0;
  int brake_multiple_ = 1;

  // chassis fsm control coefficient
  double coefficient_x_ = 0.0;
  double coefficient_y_ = 0.0;
  double coefficient_angular_ = 0.0;

  // gimbal fsm control coefficient
  double coefficient_yaw_ = 0.0;
  double coefficient_pitch_ = 0.0;

  double shoot_hz_ = 0.0;

  double lowest_effort_;

  uint8_t last_chassis_mode_;
  uint8_t last_shoot_mode_;
};

/**
 * Enumerate all of the operating modes
 */
enum class FsmOperatingMode {
  kNormal, kTransitioning, kEStop, kEDamp
};

/**
 * Control FSM handles the FSM states from a higher level
 */
template<typename T>
class Fsm {
 public:
  explicit Fsm(ros::NodeHandle &nh);

  ros::NodeHandle nh_;

  //Pointer list of each state.
  std::map<std::string, State<T> *> string2state;

  // Runs the FSM logic and handles the state transitions and normal runs
  void run();

  // Get desired state decided by control fsm data.
  virtual std::string getDesiredState() = 0;

  // Send related data to FsmState
  FsmData<T> data_;

  State<T> *current_state_;    // current FSM state
  State<T> *next_state_;       // next FSM state
  std::string next_state_name_;  // next FSM state name

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  std::string control_mode_; // pc control or rc control

 private:
  // Operating mode of the FSM
  FsmOperatingMode operating_mode_{};
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
