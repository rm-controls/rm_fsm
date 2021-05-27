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
#include "rm_fsm/state_passive.h"
#include "rm_fsm/state_calibrate.h"
#include "rm_fsm/state_standby.h"
#include "rm_fsm/state_attack.h"

namespace rm_fsm {
class FsmSentry : public Fsm {
 public:
  FsmSentry(ros::NodeHandle &nh) : Fsm(nh) {
    tf_listener_ = new tf2_ros::TransformListener(tf_);
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("move_distance", move_distance_)) {
      ROS_ERROR("Move distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!auto_nh.getParam("collision_distance", collision_distance_)) {
      ROS_ERROR("Collision distance no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    if (!auto_nh.getParam("collision", collision_)) {
      ROS_ERROR("Collision no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    string2state.insert(std::pair<std::string, State *>("passive", &state_passive_));
    string2state.insert(std::pair<std::string, State *>("raw", &state_raw_));
    string2state.insert(std::pair<std::string, State *>("calibrate", &state_calibrate_));
    string2state.insert(std::pair<std::string, State *>("standby", &state_standby_));
    string2state.insert(std::pair<std::string, State *>("attack", &state_attack_));
    current_state_ = this->string2state["passive"];
    odom2baselink_.header.frame_id = "odom";
    odom2baselink_.child_frame_id = "base_link";
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.header.frame_id = "map";
    map2odom_.child_frame_id = "odom";
    map2odom_.transform.translation.x = 0;
    map2odom_.transform.translation.y = 0;
    map2odom_.transform.translation.z = 0;
    map2odom_.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(map2odom_);
  };
  std::string getDesiredState() {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
      ros::Time time = ros::Time::now();
      if ((time - last_update_time_).toSec() > 0.1) {
        last_update_time_ = time;
        getEffort();
      }
      getPosition();
      if (finish_calibrate_) return "standby";
      else if (current_effort_ < -1.1) {
        ROS_INFO("Calibrate finish");
        finish_calibrate_ = true;
        updateTf();
        return "standby";
      } else return "calibrate";
    } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) return "raw";
    else return "passive";
  };

 protected:
  void getPosition() {
    geometry_msgs::TransformStamped odom2baselink;
    int stop_distance = 0;
    try { odom2baselink = tf_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s", ex.what()); }
    if (collision_) {
      current_speed_ = joint_state_.velocity[0];
      if ((odom2baselink.transform.translation.x >= move_distance_ - collision_distance_) && (current_position_ == 1))
        current_position_ = 2;
      else if ((odom2baselink.transform.translation.x <= collision_distance_) && (current_position_ == 3))
        current_position_ = 4;
      if (current_position_ == 2 && current_speed_ <= 0.) current_position_ = 3;
      else if (current_position_ == 4 && current_speed_ >= 0.) current_position_ = 1;
    } else {
      if (current_position_ >= move_distance_ - stop_distance - 0.2) current_position_ = 2;
      else if (current_position_ <= stop_distance) current_position_ = 1;
    }
  }
  void getEffort() {
    sum_effort_ += joint_state_.effort[0];
    current_effort_ = sum_effort_ / 10;
    if (++sum_count_ >= 10) {
      sum_count_ = 0;
      sum_effort_ = 0;
    }
  }
  void updateTf() {
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.transform.translation.x = current_position_;
    map2odom_.transform.translation.y = 0;
    map2odom_.transform.translation.z = 0;
    map2odom_.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(map2odom_);
    odom2baselink_.header.stamp = ros::Time::now();
    odom2baselink_.transform.translation.x = 0;
    odom2baselink_.transform.translation.y = 0;
    odom2baselink_.transform.translation.z = 0;
    odom2baselink_.transform.rotation.w = 1;
    tf_broadcaster_.sendTransform(odom2baselink_);
  }
  void effortDataCallback(const sensor_msgs::JointState::ConstPtr &data) { joint_state_ = *data; }

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Time last_update_time_;
  sensor_msgs::JointState joint_state_;
  geometry_msgs::TransformStamped map2odom_;
  geometry_msgs::TransformStamped odom2baselink_;
  ros::Subscriber effort_sub_ =
      nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &FsmSentry::effortDataCallback, this);

  int current_position_ = 0;
  double move_distance_, collision_distance_, current_speed_;
  double sum_effort_ = 0, sum_count_ = 0, current_effort_;
  bool collision_, finish_calibrate_ = false;
 private:
  StatePassive state_passive_ = StatePassive(nh_, &this->data_, "passive");
  StateRaw state_raw_ = StateRaw(nh_, &this->data_, "raw");
  StateCalibrate state_calibrate_ = StateCalibrate(nh_, &this->data_, "calibrate");
  StateStandby state_standby_ = StateStandby(nh_, &this->data_, "standby");
  StateAttack state_attack_ = StateAttack(nh_, &this->data_, "automatic");
};
}

#endif //RM_FSM_FSM_SENTRY_H_
