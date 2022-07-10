//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh)
    : controller_manager_(nh), context_(*this), subscriber_(context_) {
  try {
    XmlRpc::XmlRpcValue gimbal_calibration, shooter_calibration,
        catapult_calibration;
    nh.getParam("gimbal_calibration", gimbal_calibration);
    gimbal_calibration_ = new rm_common::CalibrationQueue(
        gimbal_calibration, nh, controller_manager_);
    nh.getParam("shooter_calibration", shooter_calibration);
    shooter_calibration_ = new rm_common::CalibrationQueue(
        shooter_calibration, nh, controller_manager_);
    nh.getParam("catapult_calibration", catapult_calibration);
    catapult_calibration_ = new rm_common::CalibrationQueue(
        catapult_calibration, nh, controller_manager_);
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
  ros::NodeHandle lower_nh(nh, "lower");
  lower_cmd_sender = new SideCommandSender(
      lower_nh, subscriber_.referee_, subscriber_.lower_gimbal_des_error_,
      subscriber_.pos_lower_yaw_, subscriber_.pos_lower_pitch_,
      subscriber_.lower_track_data_);

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

void StateMachine::sendGimbalCmd(bool is_auto, const DbusData &data,
                                 SideCommandSender *side_command_sender) {
  ros::Time time = ros::Time::now();
  if (subscriber_.dbus_.s_l == rm_msgs::DbusData::DOWN) {
    side_command_sender->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  if (is_auto) {
    if (side_command_sender->pos_yaw_ >= side_command_sender->yaw_max_)
      side_command_sender->yaw_direct_ = -1.;
    else if (side_command_sender->pos_yaw_ <= side_command_sender->yaw_min_)
      side_command_sender->yaw_direct_ = 1.;
    if (side_command_sender->pos_pitch_ >= side_command_sender->pitch_max_)
      side_command_sender->pitch_direct_ = -1.;
    else if (side_command_sender->pos_pitch_ <= side_command_sender->pitch_min_)
      side_command_sender->pitch_direct_ = 1.;
  } else
    side_command_sender->gimbal_cmd_sender_->setRate(-subscriber_.dbus_.ch_l_x,
                                                     -subscriber_.dbus_.ch_l_y);
  side_command_sender->gimbal_cmd_sender_->sendCommand(time);
}

void StateMachine::calibrationReset() {
  catapult_calibration_->reset();
  gimbal_calibration_->reset();
  shooter_calibration_->reset();
}

void StateMachine::update() {
  ros::Time time = ros::Time::now();
  catapult_calibration_->update(time);
  gimbal_calibration_->update(time);
  shooter_calibration_->update(time);
  controller_manager_.update();
}

void StateMachine::check() { context_.checkRc(); }
