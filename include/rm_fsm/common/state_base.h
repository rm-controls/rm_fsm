//
// Created by luotinkai on 2022/6/5.
//

#pragma once

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/common/fsm_data.h"
#include <realtime_tools/realtime_buffer.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class StateBase {
public:
  explicit StateBase(ros::NodeHandle &nh);

  void sendCommand(const ros::Time &time);

  void setChassis();

  void setShooter();

  void setGimbal();

  // referee
  void chassisOutputOn();
  void gimbalOutputOn();
  void shooterOutputOn();

  void checkReferee(const ros::Time &time);
  void checkSwitch(const ros::Time &time);

  // remote
  void remoteControlTurnOff();
  void remoteControlTurnOn();

  void run();

protected:
  rm_msgs::DbusData dbus_;
  FsmData fsm_data_;

  rm_common::CalibrationQueue *lower_trigger_calibration_{},
      *lower_gimbal_calibration_{};
  rm_common::ControllerManager controller_manager_;
  // calibrate
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  SideCommandSender *lower_cmd_sender_;
  bool chassis_output_{}, shooter_output_{}, gimbal_output_{};
  bool remote_is_open_{};
};
