//
// Created by bruce on 2021/1/17.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
#include "fsm_common.h"

template<typename T>
class StateFlySlope : public State<T> {
 public:
  StateFlySlope(FsmData<T> *fsm_data,
                const std::string &state_string,
                tf2_ros::TransformListener *tf_listener,
                ros::NodeHandle &nh,
                bool pc_control);
  void onEnter() override;
  void run() override;
  void onExit() override;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_STATE_FLYSLOPE_H_
