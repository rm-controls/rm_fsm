//
// Created by bruce on 2020/12/16.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STANDARD_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STANDARD_H_

#include "fsm_common.h"
#include "state_raw.h"
#include "state_passive.h"
#include "state_fly_slope.h"
#include "state_burst.h"

template<typename T>
class FsmStandard : public Fsm<T> {
 public:
  explicit FsmStandard(ros::NodeHandle &node_handle);
  std::string getDesiredState();
  StatePassive<T> *state_passive_;
  StateRaw<T> *state_raw_;
  StateFlySlope<T> *state_fly_slope_;
  StateBurst<T> *state_burst_;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STANDARD_H_
