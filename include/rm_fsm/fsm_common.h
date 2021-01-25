//
// Created by peter on 2020/12/3.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_

#include <rm_fsm/fsm_data.h>

#include <iostream>
#include <rm_fsm/safety_checker.h>
#include <tf/transform_listener.h>
#include <queue>
#include <ros_utilities.h>
#include <utility>

/**
 * A base fsm state class for all robots.
 * @tparam T
 */
template<typename T>
class State {
 public:
  // Generic constructor fo all states
  State(FsmData<T> *fsm_data,
        std::string state_string,
        tf2_ros::TransformListener *tf_listener,
        ros::NodeHandle &nh,
        bool pc_control);

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

  // Initializes the Control FSM instance
  void init();

  // Runs the FSM logic and handles the state transitions and normal runs
  void run();

  // FsmOperatingMode SafetyCheck();

  // Get desired state decided by control fsm data.
  virtual std::string getDesiredState() = 0;

  // Prints the current FSM status
  void printInfo(int opt);

  // Send related data to FsmState
  FsmData<T> data_;

  State<T> *current_state_;    // current FSM state
  State<T> *next_state_;       // next FSM state
  std::string next_state_name_;  // next FSM state name

  SafetyChecker<T> *safety_checker_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  bool pc_control_ = true; // pc control or rc control

 private:
  // Operating mode of the FSM
  FsmOperatingMode operating_mode_{};

  // Choose how often to print info, every N iterations
  int print_num_ = 100;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int print_iter_ = 0;  // make larger than printNum to not print

  int iter_ = 0;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
