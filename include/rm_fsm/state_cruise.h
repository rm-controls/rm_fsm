//
// Created by peter on 2021/8/3.
//

#ifndef RM_FSM_INCLUDE_STATE_CRUISE_H_
#define RM_FSM_INCLUDE_STATE_CRUISE_H_

#include <cstdlib>
#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class StateCruise : public StateStandby {
 public:
  StateCruise(ros::NodeHandle &nh, Data *data) : StateStandby(nh, data), start_pos_(0.), start_flag_(false) {
    state_name_ = "CRUISE";
    random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
  }
 protected:
  void setChassis() override {
    StateBase::setChassis();
    if (vel_2d_cmd_sender_->getMsg()->linear.x == 0.) vel_2d_cmd_sender_->setLinearXVel(1.);
    if (data_->pos_x_ <= start_pos_ - random_distance_ || data_->pos_x_ <= -move_distance_) {
      vel_2d_cmd_sender_->setLinearXVel(1.);
      if (!start_flag_) {
        start_pos_ = data_->pos_x_;
        random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
        start_flag_ = true;
      }
    } else if (data_->pos_x_ >= start_pos_ + random_distance_ || data_->pos_x_ >= 0.) {
      vel_2d_cmd_sender_->setLinearXVel(-1.);
      if (!start_flag_) {
        start_pos_ = data_->pos_x_;
        random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
        start_flag_ = true;
      }
    } else start_flag_ = false;
  }
 private:
  double random_distance_, start_pos_;
  bool start_flag_;
};
}

#endif //RM_FSM_INCLUDE_STATE_CRUISE_H_
