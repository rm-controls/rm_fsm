//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_FSM_SENTRY_H_
#define RM_FSM_FSM_SENTRY_H_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_calibrate.h"
#include "rm_fsm/state_attack.h"

namespace rm_fsm {
class FsmSentry : public Fsm {
 public:
  FsmSentry(ros::NodeHandle &nh) : Fsm(nh) {
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("move_distance", move_distance_)) {
      ROS_ERROR("Move distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!auto_nh.getParam("stop_distance", stop_distance_)) {
      ROS_ERROR("Stop distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!auto_nh.getParam("collision_distance", collision_distance_)) {
      ROS_ERROR("Collision distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!auto_nh.getParam("collision_flag", collision_flag_)) {
      ROS_ERROR("Collision flag no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    string2state.insert(std::pair<std::string, State *>("raw", &state_raw_));
    string2state.insert(std::pair<std::string, State *>("calibrate", &state_calibrate_));
    string2state.insert(std::pair<std::string, State *>("attack", &state_attack_));
    current_state_ = string2state["raw"];
    odom2baselink_.header.frame_id = "odom";
    odom2baselink_.child_frame_id = "base_link";
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.header.frame_id = "map";
    map2odom_.child_frame_id = "odom";
    map2odom_.transform.translation.x = 0.;
    map2odom_.transform.translation.y = 0.;
    map2odom_.transform.translation.z = 0.;
    map2odom_.transform.rotation.w = 1.;
    static_tf_broadcaster_.sendTransform(map2odom_);
  }
  void run() override {
    Fsm::run();
    updatePosition();
    updateMoveStatus();
    if (current_state_->getName() == "attack") current_state_->updatePosStatus(move_status_);
  }
  std::string getDesiredState() override {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
      updateEffort();
      if (finish_calibrate_) return "attack";
      else if (current_effort_ < -1.1) {
        finish_calibrate_ = true;
        move_status_ = LEAVE_START;
        updateTf();
        return "attack";
      } else return "calibrate";
    } else return "raw";
  }
 protected:
  void updatePosition() {
    geometry_msgs::TransformStamped odom2baselink;
    try { odom2baselink = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
    current_pos_x_ = odom2baselink.transform.translation.x;
  }
  void updateMoveStatus() {
    if (collision_flag_) {
      current_speed_ = joint_state_.velocity[1];
      if (move_status_ == LEAVE_START && current_pos_x_ >= move_distance_ - collision_distance_)
        move_status_ = APPROACH_END;
      else if (move_status_ == LEAVE_END && current_pos_x_ <= collision_distance_) move_status_ = APPROACH_START;
      if (move_status_ == APPROACH_END && current_speed_ <= 0.) move_status_ = LEAVE_END;
      else if (move_status_ == APPROACH_START && current_speed_ >= 0.) move_status_ = LEAVE_START;
    } else {
      if (move_status_ == LEAVE_START && current_pos_x_ >= move_distance_ - stop_distance_) move_status_ = LEAVE_END;
      else if (move_status_ == LEAVE_END && current_pos_x_ <= stop_distance_) move_status_ = LEAVE_START;
    }
  }
  void updateEffort() {
    sum_effort_ += joint_state_.effort[1];
    if (sum_count_++ >= 10) {
      current_effort_ = sum_effort_ / 10;
      sum_count_ = 1;
      sum_effort_ = 0.;
    }
  }
  void updateTf() {
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.transform.translation.x = current_pos_x_;
    map2odom_.transform.translation.y = 0.;
    map2odom_.transform.translation.z = 0.;
    map2odom_.transform.rotation.w = 1.;
    static_tf_broadcaster_.sendTransform(map2odom_);
    odom2baselink_.header.stamp = ros::Time::now();
    odom2baselink_.transform.translation.x = 0.;
    odom2baselink_.transform.translation.y = 0.;
    odom2baselink_.transform.translation.z = 0.;
    odom2baselink_.transform.rotation.w = 1.;
    tf_broadcaster_.sendTransform(odom2baselink_);
  }
  void effortDataCallback(const sensor_msgs::JointState::ConstPtr &data) { joint_state_ = *data; }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  sensor_msgs::JointState joint_state_;
  geometry_msgs::TransformStamped map2odom_;
  geometry_msgs::TransformStamped odom2baselink_;
  ros::Subscriber effort_sub_ =
      nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &FsmSentry::effortDataCallback, this);

  MoveStatus move_status_;
  double move_distance_, stop_distance_, collision_distance_, current_speed_, current_pos_x_;
  double sum_effort_ = 0, current_effort_;
  int sum_count_ = 1;
  bool collision_flag_, finish_calibrate_ = false;
 private:
  StateRaw state_raw_ = StateRaw(nh_, &data_, "raw");
  StateCalibrate state_calibrate_ = StateCalibrate(nh_, &data_, "calibrate");
  StateAttack state_attack_ = StateAttack(nh_, &data_, "attack");
};
}

#endif //RM_FSM_FSM_SENTRY_H_
