//
// Created by astro on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "rm_fsm/fsm_common.h"

template<typename T>
class StateAutomatic : public State<T> {
 public:
  StateAutomatic(FsmData<T> *fsm_data,
                 const std::string &state_string,
                 ros::NodeHandle &nh);
  void onEnter() override;
  void run() override;
  void onExit() override;
  int point_side_;
  int gimbal_position_;
  double auto_move_chassis_speed_;
  double auto_move_pitch_speed_;
  double auto_move_yaw_speed_;
  double start_;
  double end_;
  double calibration_speed_;
  int calibration_;
  int attack_id_;
  double speed_;
  double current_position_;
  ros::Time last_time_ = ros::Time::now();
  geometry_msgs::TransformStamped map2odom_;
  geometry_msgs::TransformStamped odom2baselink_;
  tf2_ros::StaticTransformBroadcaster  tf_broadcaster_;
  tf2_ros::TransformBroadcaster br;
  // sub
  sensor_msgs::JointState effort_data_;
  ros::Subscriber effort_sub_;
  void effortDataCallback(const sensor_msgs::JointState::ConstPtr &data) {
    effort_data_ = *data;
  }
};


#endif //SRC_RM_FSM_INCLUDE_RM_FSM_STATE_AUTOMATIC_H_