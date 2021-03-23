//
// Created by astro on 2021/3/23.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_ATTACK_WITHOUT_MOVE_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_STATE_ATTACK_WITHOUT_MOVE_H_

#include "rm_fsm/fsm_common.h"

template<typename T>
class StateAttackWithoutMove : public State<T> {
 public:
  StateAttackWithoutMove(FsmData<T> *fsm_data,
           const std::string &state_string,
           ros::NodeHandle &nh);
  void onEnter() override;
  void run() override;
  void onExit() override;
};

#endif
