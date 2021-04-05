//
// Created by astro on 2021/3/29.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_ESCAPE_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_ESCAPE_H_

#include "rm_fsm/fsm_common.h"

template<typename T>
class StateEscape : public State<T> {
 public:
  StateEscape(FsmData<T> *fsm_data,
              const std::string &state_string,
              ros::NodeHandle &nh);
  void onEnter() override;
  void run() override;
  void onExit() override;
};

#endif
