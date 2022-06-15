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
  void sendManualCmd(const DbusData &data);

  rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{};
  rm_common::ControllerManager controller_manager_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;

  double auto_linear_vel_{};

protected:
  StateMachineContext context_;
  Subscriber subscriber_;

  // cruise
  double safety_distance_{};
  //  std::default_random_engine random_;
  //  std::uniform_real_distribution<double> generator_{1.5, 2.0};

  // referee
  bool chassis_output_{}, shooter_output_{}, gimbal_output_{};

private:
};
