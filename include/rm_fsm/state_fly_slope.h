//
// Created by bruce on 2021/1/17.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateFlySlope : public State<T> {
 public:
  StateFlySlope(FsmData<T> *fsm_data,
                const std::string &state_string,
                ros::NodeHandle &nh);
  void onEnter() override;
  void run() override;
  void onExit() override;
  int shoot_hz;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
