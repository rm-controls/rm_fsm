//
// Created by peter on 2021/5/27.
//

#ifndef RM_FSM_STATE_CALIBRATE_H_
#define RM_FSM_STATE_CALIBRATE_H_

#include "rm_fsm/common/fsm_common.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rm_fsm {
class StateCalibrate : public StateBase {
 public:
  StateCalibrate(ros::NodeHandle &nh, Data *data) : StateBase(nh, data, "CALIBRATE") {
    ros::NodeHandle auto_nh = ros::NodeHandle(nh, "auto");
    if (!auto_nh.getParam("collision_effort", collision_effort_)) {
      ROS_ERROR("Collision effort no defined (namespace: %s)", nh.getNamespace().c_str());
    }
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
    StateBase::run();
    checkCalibrateStatus();
    if (!init_flag_) {
      init_flag_ = true;
      init_time_ = ros::Time::now();
    }
  }
  bool getCalibrateStatus() const { return finish_calibrate_; }
 protected:
  void setChassis() override {
    StateBase::setChassis();
    vel_2d_cmd_sender_->setLinearXVel(-1.);
  }
 private:
  void checkCalibrateStatus() {
    if (ros::Time::now() - init_time_ > ros::Duration(1.0) && !finish_calibrate_
        && data_->current_effort_ <= -collision_effort_) {
      finish_calibrate_ = true;
      map2odom_.header.stamp = ros::Time::now();
      map2odom_.transform.translation.x = data_->pos_x_ - 0.15;
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
  }
  double collision_effort_{};
  bool finish_calibrate_ = false, init_flag_ = false;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped map2odom_, odom2baselink_;
  ros::Time init_time_ = ros::Time::now();
};
}

#endif //RM_FSM_STATE_CALIBRATE_H_
