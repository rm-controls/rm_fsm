//
// Created by astro on 2020/12/8.
//

#include <rm_fsm/state_passive.h>

template<typename T>
StatePassive<T>::StatePassive(FsmData<T> *fsm_data,
                              const std::string &state_string,
                              ros::NodeHandle &nh):
    State<T>(fsm_data, state_string, nh) {
}

template<typename T>
void StatePassive<T>::onEnter() {
  ROS_INFO("[fsm] Enter passive mode");
}

template<typename T>
void StatePassive<T>::run() {
  double linear_x = 0, linear_y = 0, angular_z = 0;
  double rate_yaw = 0, rate_pitch = 0;
  ros::Time now = ros::Time::now();

  this->setChassis(this->data_->chassis_cmd_.PASSIVE, linear_x, linear_y, angular_z);
  this->setGimbal(this->data_->gimbal_cmd_.PASSIVE, rate_yaw, rate_pitch);
  this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, shoot_hz, now);
}

template<typename T>
void StatePassive<T>::onExit() {
//Nothing to clean up when exiting
  ROS_INFO("[fsm] Exit passive mode");
}

template
class StatePassive<double>;
template
class StatePassive<float>;