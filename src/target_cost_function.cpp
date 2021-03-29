//
// Created by kiana on 2021/3/22.
//

#include "rm_fsm/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  cost_nh.param("time_interval_", time_interval_, 0.1);

}

void TargetCostFunction::input(rm_msgs::TargetDetectionArray target_detection_array) {

  int target_numbers = target_detection_array.detections.size();
  double cost_temp;
  for (int i = 0; i < target_numbers; i++) {
    cost_temp = calculateCost(target_detection_array.detections[i]);
    if (cost_temp <= cost_) {
      cost_ = cost_temp;
      id_ = target_detection_array.detections[i].id;
    }
  }
}

double TargetCostFunction::calculateCost(rm_msgs::TargetDetection target_detection) {
  double delta_x_2 = pow(target_detection.pose.position.x, 2);
  double delta_y_2 = pow(target_detection.pose.position.y, 2);
  double delta_z_2 = pow(target_detection.pose.position.z, 2);

  double distance = sqrt(delta_x_2 + delta_y_2 + delta_z_2);
  double cost = distance * target_detection.confidence;

  return cost;
}

int TargetCostFunction::output() const {
  return id_;
}
