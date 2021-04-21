//
// Created by peter on 2021/3/15.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_FOLLOW_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_FOLLOW_H_
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateFollow : public State<T> {
 public:
  StateFollow(FsmData<T> *fsm_data,
              const std::string &state_string,
              ros::NodeHandle &nh);
  void onEnter() override;
  void run() override;
  void onExit() override;
 private:
  bool is_spin_ = false;
  bool is_friction_ready_ = false;
  bool only_attack_base_ = false;
  bool is_burst_ = false;
  bool twist_ = false;
  int last_target_id_ = 0;
  ros::Time last_press_time_g_ = ros::Time::now();
  ros::Time last_press_time_r_ = ros::Time::now();
  ros::Time last_press_time_f_ = ros::Time::now();
  ros::Time last_press_time_q_ = ros::Time::now();
  ros::Time last_press_time_c_ = ros::Time::now();
  double normal_critical_speed_;
  double burst_critical_speed_;
  double normal_angular_;
  double burst_angular_;
  double spin_sin_amplitude_;
  double spin_sin_frequency_;

  std::string robot_type_;
};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_FOLLOW_H_
