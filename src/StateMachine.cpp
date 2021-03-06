//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh)
    : controller_manager_(nh), context_(*this), subscriber_(context_) {
  try {
    XmlRpc::XmlRpcValue chassis_gimbal_calibration, shooter_calibration;
    nh.getParam("gimbal_calibration", chassis_gimbal_calibration);
    chassis_gimbal_calibration_ = new rm_common::CalibrationQueue(
        chassis_gimbal_calibration, nh, controller_manager_);
    nh.getParam("shooter_calibration", shooter_calibration);
    shooter_calibration_ = new rm_common::CalibrationQueue(
        shooter_calibration, nh, controller_manager_);
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
  if (!auto_nh.getParam("auto_linear_vel", auto_linear_vel_)) {
    ROS_ERROR("Can not find auto_linear_vel");
  }

  controller_manager_.startStateControllers();
  context_.enterStartState();
}

void StateMachine::sendChassisCmd(bool is_auto, const DbusData &data) {
  ros::Time time = ros::Time::now();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->sendCommand(time);
  if (is_auto)
    vel_2d_cmd_sender_->setLinearXVel(auto_linear_vel_);
  else
    vel_2d_cmd_sender_->setLinearXVel(data.ch_r_x);
  vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::check() { context_.checkRc(); }
