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
};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_FOLLOW_H_
