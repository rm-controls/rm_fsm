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
  void sendChassisCmd(bool is_auto, const DbusData &data);
  void catapult() { auto_linear_vel_ *= -1; }
  void calibrationReset();
  void update();
  void check();

  rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{},
      *catapult_calibration_{};
  rm_common::ControllerManager controller_manager_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;

  StateMachineContext context_;
  Subscriber subscriber_;
  double auto_linear_vel_{}, safety_distance_{};

protected:
private:
};
