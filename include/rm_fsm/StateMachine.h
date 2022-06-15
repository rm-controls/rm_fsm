//
// Created by luotinkai on 2022/2/27.
//

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/Subscriber.h"

#include <realtime_tools/realtime_buffer.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>

using namespace rm_common;

class StateMachine {
public:
  explicit StateMachine(ros::NodeHandle &nh);

  static void info(const std::string &string) { ROS_INFO_STREAM(string); }

  void checkSwitch(const ros::Time &time, rm_msgs::DbusData data);
  void remoteControlTurnOff();
  void remoteControlTurnOn();

  void changeVel() { auto_linear_x_ *= -1.; }

  // referee
  void chassisOutputOn();

  void gimbalOutputOn();

  void shooterOutputOn();

  void checkReferee(const ros::Time &time);

  // remote

  void rawRun();

  ros::Time last_time_ = ros::Time::now();
  double rand_time_ = 0.8;

protected:
  StateMachineContext context_;
  Subscriber subscriber_;

  rm_common::CalibrationQueue *calibrate_queue_{};
  rm_common::ControllerManager controller_manager_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  // cruise
  double auto_linear_x_{};
  double safety_distance_{};
  //  std::default_random_engine random_;
  //  std::uniform_real_distribution<double> generator_{1.5, 2.0};
  // referee
  bool chassis_output_{}, shooter_output_{}, gimbal_output_{};
  // remote
  bool remote_is_open_{};
};
