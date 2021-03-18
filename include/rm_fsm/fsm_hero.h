//
// Created by bruce on 2021/1/29.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_SRC_FSM_HERO_H_
#define SRC_RM_SOFTWARE_RM_FSM_SRC_FSM_HERO_H_
#include "rm_fsm/fsm_common.h"
#include "rm_fsm/state_follow.h"
#include "rm_fsm/state_passive.h"

template<typename T>
class FsmHero : public Fsm<T> {
 public:
  explicit FsmHero(ros::NodeHandle &node_handle);
  std::string getDesiredState();
  StatePassive<T> *state_passive_;
  StateFollow<T> *state_follow_;
};

#endif //SRC_RM_SOFTWARE_RM_FSM_SRC_FSM_HERO_H_
