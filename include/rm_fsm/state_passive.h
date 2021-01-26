//
// Created by astro on 2020/12/8.
//

#ifndef SRC_FSM_STATE_PASSIVE_H
#define SRC_FSM_STATE_PASSIVE_H
#include "rm_fsm/fsm_common.h"

template<typename T>
class StatePassive : public State<T> {
 public:
  StatePassive(FsmData<T> *fsm_data,
               const std::string &state_string,
               ros::NodeHandle &nh,
               bool pc_control);
  void onEnter() override;
  void run() override;
  void onExit() override;
};

#endif //SRC_FSM_STATE_PASSIVE_H
