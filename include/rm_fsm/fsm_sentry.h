//
// Created by astro on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_

#include "rm_fsm/common/fsm_common.h"
#include "state_raw.h"
#include "state_passive.h"
#include "state_automatic.h"
template<typename T>
class FsmSentry : public Fsm<T> {
 public:
  explicit FsmSentry(ros::NodeHandle &fsm_nh);
  std::string getDesiredState();
  StatePassive<T> *state_passive_;
  StateRaw<T> *state_raw_;
  StateAutomatic<T> *state_automatic_;
};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_FSM_SENTRY_H_
