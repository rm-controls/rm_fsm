//
// Created by luohx on 7/20/20.
//

#ifndef RM_BASE_RM_DECISION_SRC_FSM_SAFETY_CHECKER_H_
#define RM_BASE_RM_DECISION_SRC_FSM_SAFETY_CHECKER_H_
#include <rm_fsm/fsm_data.h>

template<typename T>
class SafetyChecker {
 public:
  explicit SafetyChecker(FsmData<T> *dataIn);

  bool checkPower();
  bool checkPos();
  bool checkTemper();
  bool checkDbusUpdate();
  FsmData<T> *data_;
 private:
  ros::Time last_dubs_stamp_{};
};

#endif //RM_BASE_RM_DECISION_SRC_FSM_SAFETY_CHECKER_H_
