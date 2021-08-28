//
// Created by luohx on 7/20/20.
//

#ifndef RM_FSM_COMMON_DATA_H_
#define RM_FSM_COMMON_DATA_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/referee/referee.h>
#include <rm_common/decision/command_sender.h>
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
    upper_track_sub_ =
        nh.subscribe<rm_msgs::TrackDataArray>("/controllers/upper_gimbal_controller/track", 10,
                                              &Data::upperTrackCallback, this);
    lower_track_sub_ =
        nh.subscribe<rm_msgs::TrackDataArray>("/controllers/lower_gimbal_controller/track", 10,
                                              &Data::lowerTrackCallback, this);
    upper_gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/controllers/upper_gimbal_controller/error_des", 10,
                                              &Data::upperGimbalDesErrorCallback, this);
    lower_gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/controllers/lower_gimbal_controller/error_des", 10,
                                              &Data::lowerGimbalDesErrorCallback, this);
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
    if (time - update_effort_ > ros::Duration(0.1)) {
      if (sum_count_ != 0) current_effort_ = sum_effort_ / sum_count_;
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
  rm_msgs::TrackDataArray upper_track_data_array_, lower_track_data_array_;
  rm_msgs::GimbalDesError upper_gimbal_des_error_, lower_gimbal_des_error_;

  rm_common::Referee referee_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double pos_x_{};
  double upper_yaw_{}, upper_pitch_{}, lower_yaw_{}, lower_pitch_{};
  double current_effort_{};
 private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state) { joint_state_ = *joint_state; }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }
  void upperTrackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { upper_track_data_array_ = *data; }
  void lowerTrackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { lower_track_data_array_ = *data; }
  void upperGimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { upper_gimbal_des_error_ = *data; }
  void lowerGimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { lower_gimbal_des_error_ = *data; }

  ros::Subscriber joint_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber upper_track_sub_, lower_track_sub_;
  ros::Subscriber upper_gimbal_des_error_sub_, lower_gimbal_des_error_sub_;
  ros::Time update_effort_;
  double sum_effort_ = 0.;
  int sum_count_ = 0;
};

class SideCommandSender {
 public:
  SideCommandSender(ros::NodeHandle &nh, rm_common::RefereeData &referee_data, rm_msgs::TrackDataArray &track_data,
                    rm_msgs::GimbalDesError &gimbal_des_error, double &pos_yaw, double &pos_pitch)
      : track_data_(track_data), gimbal_des_error_(gimbal_des_error), pos_yaw_(pos_yaw), pos_pitch_(pos_pitch) {
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, referee_data);
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, referee_data);
    ros::NodeHandle auto_nh(nh, "auto");
    XmlRpc::XmlRpcValue pitch_value, yaw_value;
    try {
      auto_nh.getParam("pitch", pitch_value);
      pitch_min_ = (double) (pitch_value[0]);
      pitch_max_ = (double) (pitch_value[1]);
      auto_nh.getParam("yaw", yaw_value);
      yaw_min_ = (double) (yaw_value[0]);
      yaw_max_ = (double) (yaw_value[1]);
    } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("%s", e.getMessage().c_str()); }
  };
  rm_common::GimbalCommandSender *gimbal_cmd_sender_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_;
  rm_msgs::TrackDataArray &track_data_;
  rm_msgs::GimbalDesError &gimbal_des_error_;
  double pitch_min_{}, pitch_max_{}, yaw_min_{}, yaw_max_{};
  double &pos_yaw_, &pos_pitch_;
  double yaw_direct_{1.}, pitch_direct_{1.};
};
}
#endif //RM_FSM_COMMON_DATA_H_