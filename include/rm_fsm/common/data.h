//
// Created by luohx on 7/20/20.
//

#ifndef RM_FSM_COMMON_DATA_H_
#define RM_FSM_COMMON_DATA_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackDataArray.h>
#include <rm_msgs/ActuatorState.h>

#include "rm_fsm/referee/referee.h"

namespace rm_fsm {

class Data {
 public:
  explicit Data(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    track_sub_ = nh.subscribe<rm_msgs::TrackDataArray>("/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/error_des", 10, &Data::gimbalDesErrorCallback, this);
    actuator_state_sub_ =
        nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &Data::actuatorStateCallback, this);
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    referee_.init();
  };
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { track_data_array_ = *data; }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { gimbal_des_error_ = *data; }
  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr &data) { actuator_state_ = *data; }

  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber actuator_state_sub_;

  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  rm_msgs::ActuatorState actuator_state_;

  Referee referee_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}
#endif //RM_FSM_COMMON_DATA_H_