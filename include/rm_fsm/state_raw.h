//
// Created by astro on 2020/12/8.
//

#ifndef SRC_FSM_STATE_RAW_H
#define SRC_FSM_STATE_RAW_H
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateRaw : public State<T> {
 public:
  StateRaw(FsmData<T> *fsm_data,
           const std::string &state_string,
           ros::NodeHandle &nh,
           const std::string &control_mode);
  void onEnter() override;
  void run() override;
  void onExit() override;
  int shoot_hz;
};

#endif //SRC_FSM_STATE_RAW_H
