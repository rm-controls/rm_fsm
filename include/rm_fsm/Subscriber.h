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
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10,
                                                &Subscriber::dbusUpdate, this);

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
    //    game_robot_status_sub_ = nh.subscribe<rm_msgs::GameRobotStatus>(
    //        "/game_robot_status", 10, &FsmData::robotGameStatusCallback,
    //        this);
    //    game_status_sub_ = nh.subscribe<rm_msgs::GameStatus>(
    //        "/game_status", 10, &FsmData::gameStatusCallback, this);
    //    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(
    //        "/joint_states", 10, &FsmData::jointStateCallback, this);
    //    left_radar_sub_ = nh.subscribe<rm_msgs::TfRadarData>(
    //        "/controllers/tf_radar_controller/left_tf_radar/data", 10,
    //        &StateMachine::leftRadarCB, this);
    //    right_radar_sub_ = nh.subscribe<rm_msgs::TfRadarData>(
    //        "/controllers/tf_radar_controller/right_tf_radar/data", 10,
    //        &StateMachine::rightRadarCB, this);
  }

  // Command sender needed;
  rm_common::RefereeData referee_;

private:
  void dbusUpdate(const DbusData::ConstPtr &data) {
    context_.dbusUpdate(*data);
  }

  StateMachineContext &context_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber left_radar_sub_, right_radar_sub_;
  ros::Subscriber lower_track_sub_, upper_track_sub_;
  ros::Subscriber referee_sub_, game_robot_status_sub_, game_status_sub_;
  ros::Subscriber joint_state_sub_;
};
