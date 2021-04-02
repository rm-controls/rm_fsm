//
// Created by astro on 2021/3/29.
//

#include "rm_fsm/state_escape.h"

template<typename T>
StateEscape<T>::StateEscape(FsmData<T> *fsm_data,
                            const std::string &state_string,
                            ros::NodeHandle &nh): State<T>(nh, fsm_data, state_string) {
}

template<typename T>
void StateEscape<T>::onEnter() {
  ROS_INFO("Enter escape mode");
}

template<typename T>
void StateEscape<T>::run() {
  ros::Time now = ros::Time::now();

  this->loadParam();

  this->setChassis(rm_msgs::ChassisCmd::RAW, 0.0, 0.0, 0.0);

  this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, 0, 0.0);

  this->setShoot(rm_msgs::ShootCmd::PUSH, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND,
                 5, now);
}
template<typename T>
void StateEscape<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit attack without move mode");
}

template
class StateEscape<double>;
template
class StateEscape<float>;
