//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_STATE_AUTOMATIC_H_
#define RM_FSM_STATE_AUTOMATIC_H_
#include "rm_fsm/common/fsm_common.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

namespace rm_fsm {
class StateAutomatic : public State {
 public:
  StateAutomatic(ros::NodeHandle &nh, Data *fsm_data, const std::string &state_string);
  void run() override;
  void effortDataCallback(const sensor_msgs::JointState::ConstPtr &data) { effort_data_ = *data; }

  ros::Subscriber effort_sub_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  tf2_ros::TransformBroadcaster br;

  ros::Time last_time_ = ros::Time::now();
  ros::Time start_calibration_time_ = ros::Time::now();

  sensor_msgs::JointState effort_data_;
  geometry_msgs::TransformStamped map2odom_;
  geometry_msgs::TransformStamped odom2baselink_;
 protected:
  int point_side_, gimbal_position_;
  double auto_move_chassis_speed_, auto_move_pitch_speed_, auto_move_yaw_speed_, auto_move_accel_x_, start_, end_,
      calibration_speed_, collision_distance_, current_speed_, current_position_;
  bool column_, calibration_finish_;
};
}

#endif //RM_FSM_STATE_AUTOMATIC_H_