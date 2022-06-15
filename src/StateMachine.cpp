//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh)
    : context_(*this), subscriber_(context_), controller_manager_(nh) {
  try {
    XmlRpc::XmlRpcValue calibrate_queue;
    nh.getParam("calibrate_queue", calibrate_queue);
    calibrate_queue_ = new rm_common::CalibrationQueue(calibrate_queue, nh,
                                                       controller_manager_);
  } catch (XmlRpc::XmlRpcException &e) {
    ROS_ERROR("%s", e.getMessage().c_str());
  }
  ros::NodeHandle chassis_nh(nh, "chassis");
  if (!chassis_nh.getParam("safety_distance", safety_distance_))
    ROS_ERROR("no definition of safety_distance");

  chassis_cmd_sender_ =
      new rm_common::ChassisCommandSender(chassis_nh, subscriber_.referee_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);

  ros::NodeHandle auto_nh(nh, "auto");
  if (!auto_nh.getParam("auto_linear_x", auto_linear_x_)) {
    ROS_ERROR("Can not find auto_linear_x");
  }

  controller_manager_.startStateControllers();
  context_.enterStartState();
}

void StateMachine::checkSwitch(const ros::Time &time, rm_msgs::DbusData data) {
  if (remote_is_open_ && (time - data.stamp).toSec() > 0.3) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data.stamp).toSec() < 0.3) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
}

// void StateMachine::chassisOutputOn() {
//   ROS_INFO("Chassis output ON");
//   catapult_calibration_->reset();
// }
//
// void StateMachine::gimbalOutputOn() {
//   ROS_INFO("Gimbal output ON");
//   lower_gimbal_calibration_->reset();
//   upper_gimbal_calibration_->reset();
// }
//
// void StateMachine::shooterOutputOn() {
//   ROS_INFO("Shooter output ON");
//   lower_trigger_calibration_->reset();
//   upper_trigger_calibration_->reset();
// }

void StateMachine::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
}

void StateMachine::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
}
