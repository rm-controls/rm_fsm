//
// Created by astro on 2020/12/8.
//

#ifndef RM_FSM_STATE_PASSIVE_H
#define RM_FSM_STATE_PASSIVE_H
#include "rm_fsm/common/fsm_common.h"

namespace rm_fsm {
class StatePassive : public State {
 public:
  StatePassive(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string) :
      State(nh, fsm_data, state_string) {}
};
}

#endif //RM_FSM_STATE_PASSIVE_H
