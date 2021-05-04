//
// Created by kiana on 2021/3/22.
//

#include "rm_fsm/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  cost_nh.param("k_f", k_f_, 0.0);
  id_ = 0;
  time_interval_ = 0.01;
}

void TargetCostFunction::input(rm_msgs::TrackDataArray track_data_array, bool only_attack_base) {
  int target_numbers = track_data_array.tracks.size();
  int id_temp;
  double cost_temp;
  decide_old_target_time_ = ros::Time::now();

  if (target_numbers) {
    for (int i = 0; i < target_numbers; i++) {
      cost_temp = calculateCost(track_data_array.tracks[i]);
      if (cost_temp <= calculate_cost_) {
        // detective a target near than last target,change target
        calculate_cost_ = cost_temp;
        id_temp = track_data_array.tracks[i].id;
      }
      if (only_attack_base && track_data_array.tracks[i].id == 8) {
        // enter only attack base mode,can not detective base,choose to attack sentry if we can detective
        id_ = 8;
      }
      if (only_attack_base && track_data_array.tracks[i].id == 9) {
        // enter only attack base mode, detective base
        id_ = 9;
        break;
      }
    }


    //maybe we can consider frequency at this part if did not enter attack base mode
    if (!only_attack_base && id_temp != id_) {
      decide_new_target_time_ = ros::Time::now();
      time_interval_ = time_interval_ + (decide_new_target_time_ - decide_old_target_time_).toSec();
      double judge = calculate_cost_ + k_f_ / time_interval_;
      if (judge <= choose_cost_) {
        id_ = id_temp;
        time_interval_ = 0.0;
      }
      if (id_ == 0) id_ = id_temp;
    }
    calculate_cost_ = 1000000.0;
    choose_cost_ = (!only_attack_base && id_ == id_temp) ? calculate_cost_ : choose_cost_;

  } else id_ = 0;

}

double TargetCostFunction::calculateCost(rm_msgs::TrackData track_data) {
  /*
  double delta_x_2 = pow(track_data.pose.position.x + time_interval_ * track_data.speed.linear.x, 2);
  double delta_y_2 = pow(track_data.pose.position.y + time_interval_ * track_data.speed.linear.y, 2);
  double delta_z_2 = pow(track_data.pose.position.z + time_interval_ * track_data.speed.linear.z, 2);
   */

  //not speed
  double delta_x_2 = pow(track_data.map2detection.position.x, 2);
  double delta_y_2 = pow(track_data.map2detection.position.y, 2);
  double delta_z_2 = pow(track_data.map2detection.position.z, 2);

  double distance = sqrt(delta_x_2 + delta_y_2 + delta_z_2);
  double cost = distance;

  return cost;
}

int TargetCostFunction::output() const {
  return id_;
}
