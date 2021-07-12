//
// Created by luohx on 7/20/20.
//

#ifndef RM_FSM_COMMON_DATA_H_
#define RM_FSM_COMMON_DATA_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/referee/referee.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackDataArray.h>
#include <rm_msgs/ActuatorState.h>

namespace rm_fsm {

class Data {
 public:
  explicit Data(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    track_sub_ = nh.subscribe<rm_msgs::TrackDataArray>("/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/error_des", 10, &Data::gimbalDesErrorCallback, this);
    actuator_state_sub_ =
        nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 100, &Data::actuatorStateCallback, this);
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    referee_.init();
  };
  void update(const ros::Time &time) {
    geometry_msgs::TransformStamped odom2baselink, baselink2pitch, baselink2yaw;
    double roll, pitch, yaw;
    referee_.read();
    sum_effort_ += actuator_state_.effort[2];
    sum_count_++;
    if (time - update_effort_ > ros::Duration(0.2)) {
      current_effort_ = sum_effort_ / sum_count_;
      sum_effort_ = 0.;
      sum_count_ = 0;
    }
    try { odom2baselink = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
    pos_x_ = odom2baselink.transform.translation.x;
    try { baselink2pitch = tf_buffer_.lookupTransform("base_link", "pitch", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_ERROR("%s", ex.what()); }
    quatToRPY(baselink2pitch.transform.rotation, roll, pitch, yaw);
    pos_pitch_ = pitch;
    try { baselink2yaw = tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_ERROR("%s", ex.what()); }
    quatToRPY(baselink2yaw.transform.rotation, roll, pitch, yaw);
    pos_yaw_ = yaw;
  }

  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  rm_msgs::ActuatorState actuator_state_;
  rm_common::Referee referee_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double pos_x_, pos_pitch_, pos_yaw_;
  double current_effort_ = 0;
 private:
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { track_data_array_ = *data; }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { gimbal_des_error_ = *data; }
  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr &data) { actuator_state_ = *data; }

  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Time update_effort_;
  double sum_effort_ = 0.;
  int sum_count_ = 0;
};
}
#endif //RM_FSM_COMMON_DATA_H_