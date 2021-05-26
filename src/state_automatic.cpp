//
// Created by astro on 2021/1/26.
//

#include "rm_fsm/state_automatic.h"

namespace rm_fsm {
StateAutomatic::StateAutomatic(ros::NodeHandle &fsm_nh, Data *fsm_data, const std::string &state_string)
    : State(fsm_nh, fsm_data, state_string) {
  tf_listener_ = new tf2_ros::TransformListener(tf_);
  point_side_ = 1;
  gimbal_position_ = 1;
  calibration_finish_ = false;
  current_speed_ = 0;
  current_position_ = 0;
  map2odom_.header.stamp = ros::Time::now();
  map2odom_.header.frame_id = "map";
  map2odom_.child_frame_id = "odom";
  map2odom_.transform.translation.x = 0;
  map2odom_.transform.translation.y = 0;
  map2odom_.transform.translation.z = 0;
  map2odom_.transform.rotation.w = 1;
  tf_broadcaster_.sendTransform(map2odom_);
  effort_sub_ =
      fsm_nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &StateAutomatic::effortDataCallback, this);
}

void StateAutomatic::run() {
  geometry_msgs::TransformStamped gimbal_transformStamped;
  geometry_msgs::TransformStamped chassis_transformStamped;
  double now_effort = 0;
  static double sum_effort = 0;
  static int time_counter2 = 0;
  double roll{}, pitch{}, yaw{};
  ros::Time now = ros::Time::now();
  double stop_distance = 0.5 * auto_move_chassis_speed_ * auto_move_chassis_speed_ / auto_move_accel_x_;

  try {
    gimbal_transformStamped = tf_.lookupTransform("yaw", "pitch", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {}
  quatToRPY(gimbal_transformStamped.transform.rotation, roll, pitch, yaw);
  try {
    chassis_transformStamped = tf_.lookupTransform("odom", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {}
  sum_effort += effort_data_.effort[0];
  time_counter2++;
  if (time_counter2 == 10) {
    current_position_ = chassis_transformStamped.transform.translation.x;
    current_speed_ = effort_data_.velocity[0];
    now_effort = sum_effort / 10.0;
    time_counter2 = 0;
    sum_effort = 0;
  }

  if (calibration_finish_) {
    if (column_) {
      if ((current_position_ >= end_ - collision_distance_) && (point_side_ == 1)) point_side_ = 2;
      else if ((current_position_ <= start_ + collision_distance_) && (point_side_ == 3)) point_side_ = 4;
      if (point_side_ == 1) {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        vel_cmd_sender_->setLinearXVel(auto_move_chassis_speed_);
      } else if (point_side_ == 2) {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
        if (current_speed_ <= 0) point_side_ = 3;
      } else if (point_side_ == 3) {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        vel_cmd_sender_->setLinearXVel(-auto_move_chassis_speed_);
      } else if (point_side_ == 4) {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
        if (current_speed_ >= 0) point_side_ = 1;
      }
    } else {
      if (current_position_ >= end_ - stop_distance - 0.2) point_side_ = 2;
      else if (current_position_ <= start_ + stop_distance) point_side_ = 1;
      if (point_side_ == 1) {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        vel_cmd_sender_->setLinearXVel(auto_move_chassis_speed_);
      } else {
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        vel_cmd_sender_->setLinearXVel(-auto_move_chassis_speed_);
      }
    }
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_->track_data_array_, false);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkGimbalError(data_->gimbal_des_error_.error);
  } else {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    vel_cmd_sender_->setLinearXVel(-calibration_speed_);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::PASSIVE);
    if (now - start_calibration_time_ > ros::Duration(0.4)) {
      if (now_effort < -1.1) {
        ROS_INFO("calibration finish !");
        calibration_finish_ = 1;
        map2odom_.header.stamp = ros::Time::now();
        map2odom_.header.frame_id = "map";
        map2odom_.child_frame_id = "odom";
        map2odom_.transform.translation.x = current_position_;
        map2odom_.transform.translation.y = 0;
        map2odom_.transform.translation.z = 0;
        map2odom_.transform.rotation.w = 1;
        tf_broadcaster_.sendTransform(map2odom_);

        odom2baselink_.header.stamp = ros::Time::now();
        odom2baselink_.header.frame_id = "odom";
        odom2baselink_.child_frame_id = "base_link";
        odom2baselink_.transform.translation.x = 0;
        odom2baselink_.transform.translation.y = 0;
        odom2baselink_.transform.translation.z = 0;
        odom2baselink_.transform.rotation.w = 1;
        br.sendTransform(odom2baselink_);
      }
    }
  }
  sendCommand(now);
}
}