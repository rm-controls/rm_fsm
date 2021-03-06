//
// Created by qiayuan on 2022/6/15.
//
#pragma once

#include "StateMachine.h"

#include <rm_common/referee/data.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/Referee.h>
#include <rm_msgs/TofRadarData.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class Subscriber {
public:
  explicit Subscriber(StateMachineContext &fsm_context)
      : context_(fsm_context) {
    ros::NodeHandle nh;
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
        "/dbus_data", 10, &Subscriber::dbusCallback, this);
    game_robot_status_sub_ = nh.subscribe<rm_msgs::GameRobotStatus>(
        "/game_robot_status", 10, &Subscriber::robotGameStatusCallback, this);
    //    game_status_sub_ = nh.subscribe<rm_msgs::GameStatus>(
    //        "/game_status", 10, &Subscriber::gameStatusCallback, this);
    left_radar_sub_ = nh.subscribe<rm_msgs::TofRadarData>(
        "/controllers/tf_radar_controller/left_tf_radar/data", 10,
        &Subscriber::leftRadarCallback, this);
    right_radar_sub_ = nh.subscribe<rm_msgs::TofRadarData>(
        "/controllers/tf_radar_controller/right_tf_radar/data", 10,
        &Subscriber::rightRadarCallback, this);

    //    lower_track_sub_ = nh.subscribe<rm_msgs::TrackData>(
    //        "/controllers/lower_gimbal_controller/track", 10,
    //        &FsmData::lowerTrackCallback, this);
    //    upper_track_sub_ = nh.subscribe<rm_msgs::TrackData>(
    //        "/controllers/upper_gimbal_controller/track", 10,
    //        &FsmData::upperTrackCallback, this);
    //    lower_gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
    //        "/controllers/lower_gimbal_controller/error_des", 10,
    //        &FsmData::lowerGimbalDesErrorCallback, this);
    //    upper_gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
    //        "/controllers/upper_gimbal_controller/error_des", 10,
    //        &FsmData::upperGimbalDesErrorCallback, this);

    //    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(
    //        "/joint_states", 10, &FsmData::jointStateCallback, this);
  }

  rm_common::RefereeData
      referee_; // Only contains chassis power info for chassis sender;

  rm_msgs::DbusData dbus_;

private:
  void dbusCallback(const DbusData::ConstPtr &data) {
    context_.dbusUpdate(*data);
  }
  void robotGameStatusCallback(const GameRobotStatus::ConstPtr &data) {
    context_.robotStatusUpdate(*data);
    // Actually referee act as a buffer for the robot game status
    // TODO: adding heat of shooter
    referee_.game_robot_status_.mains_power_chassis_output_ =
        data->mains_power_chassis_output;
    referee_.game_robot_status_.mains_power_gimbal_output_ =
        data->mains_power_gimbal_output;
    referee_.game_robot_status_.mains_power_shooter_output_ =
        data->mains_power_shooter_output;

    referee_.game_robot_status_.chassis_power_limit_ =
        data->chassis_power_limit;
    referee_.game_robot_status_.robot_id_ = data->robot_id;
  }
  //  void gameStatusCallback(const GameStatus::ConstPtr &data) {
  //    context_.refereeUpdate(*data);
  //  }
  void leftRadarCallback(const TofRadarData::ConstPtr &data) {
    context_.leftRadarCB(*data);
  }
  void rightRadarCallback(const TofRadarData::ConstPtr &data) {
    context_.rightRadarCB(*data);
  }

  StateMachineContext &context_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber left_radar_sub_, right_radar_sub_;
  ros::Subscriber lower_track_sub_, upper_track_sub_;
  ros::Subscriber referee_sub_, game_robot_status_sub_, game_status_sub_;
  ros::Subscriber joint_state_sub_;
};
