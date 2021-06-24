//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_FSM_SENTRY_H_
#define RM_FSM_FSM_SENTRY_H_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_calibrate.h"
#include "rm_fsm/state_attack.h"

namespace rm_fsm {
class FsmSentry : public FsmBase {
 public:
  FsmSentry(ros::NodeHandle &nh) : FsmBase(nh) {
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("collision_effort", collision_effort_)) {
      ROS_ERROR("Collision effort no defined (namespace: %s)", nh.getNamespace().c_str());
    }
    string2state.insert(std::pair<std::string, StateBase *>("raw", &state_raw_));
    string2state.insert(std::pair<std::string, StateBase *>("calibrate", &state_calibrate_));
    string2state.insert(std::pair<std::string, StateBase *>("attack", &state_attack_));
    string2state.insert(std::pair<std::string, StateBase *>("passive", &state_passive_));
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
 protected:
  std::string getNextState() override {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
      if (finish_calibrate_) return "attack";
      if (!finish_calibrate_ && data_.current_effort_ <= -collision_effort_) {
        finish_calibrate_ = true;
        broadcastTf();
        return "attack";
      } else return "calibrate";
    } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) return "raw";
    else return "passive";
  }
  void broadcastTf() {
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.transform.translation.x = data_.pos_x_;
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

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped map2odom_, odom2baselink_;
  double collision_effort_;
  bool finish_calibrate_ = false;
 private:
  StateBase state_passive_ = StateBase(nh_, &data_, "passive");
  StateRaw state_raw_ = StateRaw(nh_, &data_, "raw");
  StateCalibrate state_calibrate_ = StateCalibrate(nh_, &data_, "calibrate");
  StateAttack state_attack_ = StateAttack(nh_, &data_, "attack");
};
}

#endif //RM_FSM_FSM_SENTRY_H_
