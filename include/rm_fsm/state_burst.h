//
// Created by bruce on 2021/1/17.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_BURST_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_BURST_H_
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateBurst : public State<T> {
 public:
  StateBurst(FsmData<T> *fsm_data,
             const std::string &state_string,
             ros::NodeHandle &nh,
             bool pc_control);
  void onEnter() override;
  void run() override;
  void onExit() override;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_BURST_H_
