//
// Created by kiana on 2021/3/22.
//

#include "rm_fsm/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  cost_nh.param("k_f", k_f_, 0.0);
  cost_nh.param("k_hp", k_hp_, 0.0);
  cost_nh.param("track_msg_timeout", track_msg_timeout_, 1.0);
  cost_nh.param("enemy_color", enemy_color_, std::string("error"));
  cost_nh.param("time_interval", time_interval_, 0.1);
  cost_nh.param("cost_clean_time", cost_clean_time_, 1.0);
  id_ = 0;
}

void TargetCostFunction::input(rm_msgs::TrackDataArray track_data_array, GameRobotHp robot_hp, bool only_attack_base) {
  /*
  double timeout_judge = (ros::Time::now() - track_data_array.header.stamp).toSec();
  if (timeout_judge > track_msg_timeout_) {
    id_ = 0;
  }
  else decideId(track_data_array, robot_hp, only_attack_base);
   */
  decideId(track_data_array, robot_hp, only_attack_base);
}

void TargetCostFunction::decideId(rm_msgs::TrackDataArray track_data_array,
                                  GameRobotHp robot_hp,
                                  bool only_attack_base) {
  double track_number = track_data_array.tracks.size();

  //clean cost
  if ((ros::Time::now() - last_clean_time_).toSec() > cost_clean_time_) {
    this->cleanCost(track_data_array);
    last_clean_time_ = ros::Time::now();
    ROS_INFO("clean");
  }

  if (!track_number) id_ = 0;
  else {
    //Update targets cost
    for (int i = 0; i < track_number; i++) {
      if (track_data_array.tracks[i].id == 1) cost_[0] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (track_data_array.tracks[i].id == 2) cost_[1] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (track_data_array.tracks[i].id == 3) cost_[2] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (track_data_array.tracks[i].id == 4) cost_[3] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (track_data_array.tracks[i].id == 5) cost_[4] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (track_data_array.tracks[i].id == 7) cost_[6] = calculateCost(track_data_array.tracks[i], robot_hp);
      else if (only_attack_base && track_data_array.tracks[i].id == 8) id_ = 8;
      else if ((only_attack_base && track_data_array.tracks[i].id == 9)) id_ = 9;
      else
        ROS_INFO("Detect a non-existent id:%d", track_data_array.tracks[i].id);
    }

    //find min cost
    int id_temp = 1;
    double cost_temp = cost_[0];
    for (int j = 1; j < 7; j++) {
      cost_temp = (cost_temp < cost_[j]) ? cost_temp : cost_[j];
      id_temp = (cost_temp < cost_[j]) ? id_temp : j + 1;
    }

    //decide change id or not
    if (id_ != 9 && id_ != id_temp) {
      if (id_ == 0) id_ = id_temp;
      else {
        cost_[id_temp - 1] += k_f_ / time_interval_;
        id_ = (cost_[id_ - 1] < cost_[id_temp - 1]) ? id_ : id_temp;
        time_interval_ = (cost_[id_ - 1] < cost_[id_temp - 1]) ? (time_interval_ + 0.1) : 0.1;
      }
    }
  }

}


double TargetCostFunction::calculateCost(rm_msgs::TrackData track_data, GameRobotHp robot_hp) {
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

  //Hp
  double hp_cost;
  if (enemy_color_ == "red") {
    if (track_data.id == 1) hp_cost = robot_hp.red_1_robot_HP;
    else if (track_data.id == 2) hp_cost = robot_hp.red_2_robot_HP;
    else if (track_data.id == 3) hp_cost = robot_hp.red_3_robot_HP;
    else if (track_data.id == 4) hp_cost = robot_hp.red_4_robot_HP;
    else if (track_data.id == 5) hp_cost = robot_hp.red_5_robot_HP;
    else if (track_data.id == 7) hp_cost = robot_hp.red_7_robot_HP;
    else hp_cost = 0.0;
  } else if (enemy_color_ == "blue") {
    if (track_data.id == 1) hp_cost = robot_hp.blue_1_robot_HP;
    else if (track_data.id == 2) hp_cost = robot_hp.blue_2_robot_HP;
    else if (track_data.id == 3) hp_cost = robot_hp.blue_3_robot_HP;
    else if (track_data.id == 4) hp_cost = robot_hp.blue_4_robot_HP;
    else if (track_data.id == 5) hp_cost = robot_hp.blue_5_robot_HP;
    else if (track_data.id == 7) hp_cost = robot_hp.blue_7_robot_HP;
    else hp_cost = 0.0;
  } else hp_cost = 0.0;

  double cost = distance - k_hp_ * hp_cost;

  return cost;
}

void TargetCostFunction::cleanCost(rm_msgs::TrackDataArray track_data_array) {
  int track_id;
  bool id_flag[7] = {false, false, false, false, false, false, false};

  for (int i = 0; i < int(track_data_array.tracks.size()); i++) {
    track_id = track_data_array.tracks[i].id;
    if (track_id < 8 && track_id != 6) {
      id_flag[track_id - 1] = true;
    }
  }

  for (int j = 0; j < 7; j++) {
    cost_[j] = id_flag[j] ? cost_[j] : 999999;
  }

}

int TargetCostFunction::output() const {
  return id_;
}
