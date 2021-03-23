//
// Created by kiana on 2021/3/22.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include "rm_msgs/TargetDetection.h"
#include "rm_msgs/TargetDetectionArray.h"

class TargetCostFunction {
 public:
  explicit TargetCostFunction(ros::NodeHandle &nh);
  void input(rm_msgs::TargetDetectionArray target_detection_array);
  int output() const;
  double calculateCost(rm_msgs::TargetDetection target_detection);

 private:
  int id_;
  double cost_ = 1000000;
  double time_interval_;
  geometry_msgs::Pose pose_self_;
  ros::Subscriber self_pose_sub_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  geometry_msgs::TransformStamped gimbal_transformStamped;

};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
