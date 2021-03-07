//
// Created by peter on 2020/12/3.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_

#include <rm_fsm/fsm_data.h>

#include <iostream>
#include <tf/transform_listener.h>
#include <queue>
#include <ros_utilities.h>
#include <utility>
#include <control_toolbox/pid.h>
#include "ori_tool.h"
#include "rm_fsm/safety_checker.h"

/**
 * A base fsm state class for all robots.
 * @tparam T
 */
template<typename T>
class State {
 public:
  // Generic constructor fo all states
  State(FsmData<T> *fsm_data, std::string state_string, ros::NodeHandle &nh, bool pc_control);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;

  // Run the normal behavior for the state
  virtual void run() = 0;

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0;

  // Base controllers.
  void setChassis(uint8_t, double, double, double);
  void setGimbal(uint8_t, double, double);
  void setShoot(uint8_t, uint8_t, double, ros::Time);

  // Holds all of the relevant control data
  FsmData<T> *data_;

  // FSM State info
  std::string state_name_;     // enumerated name of the current state

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  ros::NodeHandle state_nh_;

  bool pc_control_;

  // chassis fsm control accelerate
  double accel_x_ = 0.0;
  double accel_y_ = 0.0;
  double accel_angular_ = 0.0;

  // chassis fsm control coefficient
  double coefficient_x_ = 0.0;
  double coefficient_y_ = 0.0;
  double coefficient_angular_ = 0.0;

  // gimbal fsm control coefficient
  double coefficient_yaw_ = 0.0;
  double coefficient_pitch_ = 0.0;
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

  // FsmOperatingMode SafetyCheck();

  // Get desired state decided by control fsm data.
  virtual std::string getDesiredState() = 0;

  // Send related data to FsmState
  FsmData<T> data_;

  State<T> *current_state_;    // current FSM state
  State<T> *next_state_;       // next FSM state
  std::string next_state_name_;  // next FSM state name

  SafetyChecker<T> *safety_checker_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  bool pc_control_; // pc control or rc control

 private:
  // Operating mode of the FSM
  FsmOperatingMode operating_mode_{};
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
