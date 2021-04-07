//
// Created by kiana on 2021/3/22.
//

#include "rm_fsm/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  cost_nh.param("time_interval_", time_interval_, 0.1);

}

void TargetCostFunction::input(rm_msgs::TrackDataArray track_data_array) {

  int target_numbers = track_data_array.tracks.size();
  double cost_temp;
  if (target_numbers) {
    for (int i = 0; i < target_numbers; i++) {
      cost_temp = calculateCost(track_data_array.tracks[i]);
      if (cost_temp <= cost_) {
        cost_ = cost_temp;
        id_ = track_data_array.tracks[i].id;
      }
    }
    cost_ = 10000000;
  } else id_ = 0;

}

double TargetCostFunction::calculateCost(rm_msgs::TrackData track_data) {
  /*
  double delta_x_2 = pow(track_data.pose.position.x + time_interval_ * track_data.speed.linear.x, 2);
  double delta_y_2 = pow(track_data.pose.position.y + time_interval_ * track_data.speed.linear.y, 2);
  double delta_z_2 = pow(track_data.pose.position.z + time_interval_ * track_data.speed.linear.z, 2);
   */

  //not speed
  double delta_x_2 = pow(track_data.pose2map.position.x, 2);
  double delta_y_2 = pow(track_data.pose2map.position.y, 2);
  double delta_z_2 = pow(track_data.pose2map.position.z, 2);

  double distance = sqrt(delta_x_2 + delta_y_2 + delta_z_2);
  double cost = distance;

  return cost;
}

int TargetCostFunction::output() const {
  return id_;
}
