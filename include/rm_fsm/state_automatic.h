//
// Created by astro on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateAutomatic : public State<T> {
 public:
  StateAutomatic(FsmData<T> *fsm_data,
           const std::string &state_string,
           ros::NodeHandle &nh,
           bool pc_control);
  void onEnter() override;
  void run() override;
  void onExit() override;
  int point_side_;
  int gimbal_position_;
  double auto_move_chassis_speed_;
  double auto_move_chassis_accel_;
  double auto_move_pitch_speed_;
  double auto_move_yaw_speed_;
  double auto_move_distance_;
};


#endif //SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_
