//
// Created by astro on 2020/12/8.
//

#include <rm_fsm/state_passive.h>

template<typename T>
StatePassive<T>::StatePassive(FsmData<T> *fsm_data,
                              const std::string &state_string,
                              ros::NodeHandle &nh):
    State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StatePassive<T>::onEnter() {
  ROS_INFO("Enter passive mode");
}

template<typename T>
void StatePassive<T>::run() {
  ros::Time now = ros::Time::now();

  this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
  this->setGimbal(rm_msgs::GimbalCmd::PASSIVE, 0.0, 0.0, 0);
  this->setShoot(rm_msgs::ShootCmd::STOP, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, 0.0, now);
}

template<typename T>
void StatePassive<T>::onExit() {
//Nothing to clean up when exiting
  ROS_INFO("Exit passive mode");
}

template
class StatePassive<double>;
template
class StatePassive<float>;