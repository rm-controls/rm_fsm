//
// Created by luohx on 7/20/20.
//

#ifndef RM_FSM_COMMON_DATA_H_
#define RM_FSM_COMMON_DATA_H_

#include "rm_fsm/referee/referee.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackDataArray.h>

namespace rm_fsm {

class Data {
 public:
  explicit Data(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Data::jointStateCallback, this);
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    track_sub_ =
        nh.subscribe<rm_msgs::TrackDataArray>("/controllers/gimbal_controller/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/controllers/gimbal_controller/error_des", 10,
                                              &Data::gimbalDesErrorCallback, this);
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.init();
  };
  void update(const ros::Time &time) {
    geometry_msgs::TransformStamped odom2baselink;
    referee_.read();
    if (!joint_state_.effort.empty())
      sum_effort_ += joint_state_.effort[0];
    sum_count_++;
    if (time - update_effort_ > ros::Duration(0.2)) {
      current_effort_ = sum_effort_ / sum_count_;
      sum_effort_ = 0.;
      sum_count_ = 0;
    }
    try { odom2baselink = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_ERROR_ONCE("%s", ex.what()); }
    pos_x_ = odom2baselink.transform.translation.x;
    if (!joint_state_.position.empty()) {
      upper_yaw_ = joint_state_.position[10];
      upper_pitch_ = joint_state_.position[7];
      lower_yaw_ = joint_state_.position[5];
      lower_pitch_ = joint_state_.position[2];
    }
  }

  sensor_msgs::JointState joint_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;

  Referee referee_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double pos_x_{};
  double upper_yaw_{}, upper_pitch_{}, lower_yaw_{}, lower_pitch_{};
  double current_effort_{};
 private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state) { joint_state_ = *joint_state; }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { track_data_array_ = *data; }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { gimbal_des_error_ = *data; }

  ros::Subscriber joint_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Time update_effort_;
  double sum_effort_ = 0.;
  int sum_count_ = 0;
};
}
#endif //RM_FSM_COMMON_DATA_H_