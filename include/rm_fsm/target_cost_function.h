//
// Created by kiana on 2021/3/22.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "rm_msgs/TrackData.h"

class TargetCostFunction {
 public:
  TargetCostFunction(ros::NodeHandle &nh);
  void input(rm_msgs::TrackData track_data);
  int output() const;
  double getCost(geometry_msgs::Twist twist, geometry_msgs::Pose pose);

 private:
  int id_;
  double cost_ = 1000000;
  double time_interval_;
  geometry_msgs::Pose pose_self_;

};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
