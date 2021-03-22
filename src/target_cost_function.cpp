//
// Created by kiana on 2021/3/22.
//

#include "rm_fsm/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");

  cost_nh.param("time_interval_", time_interval_, 0.1);

}

void TargetCostFunction::input(rm_msgs::TrackData track_data) {
  int total_number = track_data.name.size();
  double cost_temp = 0;
  for (int i = 0; i < total_number; i++) {
    cost_temp = getCost(track_data.speed[i], track_data.pose[i]);
    if (cost_temp <= cost_) {
      cost_ = cost_temp;
      id_ = i;
    }
  }
}

double TargetCostFunction::getCost(geometry_msgs::Twist twist, geometry_msgs::Pose pose) {
  //0.1s later ,the distance of target and robot
  double target_later_pose_x = pose.position.x + time_interval_ * twist.linear.x;
  double target_later_pose_y = pose.position.y + time_interval_ * twist.linear.y;
  double target_later_pose_z = pose.position.z + time_interval_ * twist.linear.z;
  double delta_x_2 = pow(pose_self_.position.x - target_later_pose_x, 2);
  double delta_y_2 = pow(pose_self_.position.y - target_later_pose_y, 2);
  double delta_z_2 = pow(pose_self_.position.z - target_later_pose_z, 2);

  double cost = sqrt(delta_x_2 + delta_y_2 + delta_z_2);

  return cost;
}

int TargetCostFunction::output() const {
  return id_;
}