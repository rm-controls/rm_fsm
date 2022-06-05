//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh)
    : context_(*this), controller_manager_(nh) {
  try {
    XmlRpc::XmlRpcValue lower_trigger_rpc_value, lower_gimbal_rpc_value;
    nh.getParam("lower_trigger_calibration", lower_trigger_rpc_value);
    nh.getParam("lower_gimbal_calibration", lower_gimbal_rpc_value);
    lower_trigger_calibration_ = new rm_common::CalibrationQueue(
        lower_trigger_rpc_value, nh, controller_manager_);
    lower_gimbal_calibration_ = new rm_common::CalibrationQueue(
        lower_gimbal_rpc_value, nh, controller_manager_);
  } catch (XmlRpc::XmlRpcException &e) {
    ROS_ERROR("%s", e.getMessage().c_str());
  }
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(
      chassis_nh, fsm_data_.referee_.referee_data_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  ros::NodeHandle lower_nh(nh, "lower");
  lower_cmd_sender_ = new SideCommandSender(
      lower_nh, fsm_data_.referee_.referee_data_,
      fsm_data_.lower_track_data_array_, fsm_data_.lower_gimbal_des_error_,
      fsm_data_.lower_yaw_, fsm_data_.lower_pitch_);
  ros::NodeHandle auto_nh(nh, "auto");
  if (!auto_nh.getParam("auto_linear_x", auto_linear_x_)) {
    ROS_ERROR("Can not find auto_linear_x");
  }
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10,
                                              &StateMachine::dbusCB, this);
  left_radar_sub_ = nh.subscribe<rm_msgs::TfRadarData>(
      "/controllers/tf_radar_controller/left_tf_radar/data", 10,
      &StateMachine::leftRadarCB, this);
  right_radar_sub_ = nh.subscribe<rm_msgs::TfRadarData>(
      "/controllers/tf_radar_controller/right_tf_radar/data", 10,
      &StateMachine::rightRadarCB, this);

  controller_manager_.startStateControllers();
  context_.enterStartState();
}

void StateMachine::update(const ros::Time &time) {
  ros::Time begin_time = ros::Time::now();
  if ((begin_time - last_time_).toSec() >= rand_time_) {
    changeVel();
    last_time_ = ros::Time::now();
    rand_time_ = generator_(random_);
  }
}

void StateMachine::initRaw() {
  ROS_INFO("Enter Raw");
  lower_cmd_sender_->gimbal_cmd_sender_->setRate(0, 0);
  lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::initCruise() { ROS_INFO("Enter Cruise"); }

void StateMachine::sendRawCommand(const ros::Time &time) {
  vel_2d_cmd_sender_->setLinearXVel(dbus_.ch_r_x);
  vel_2d_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
}

void StateMachine::sendCruiseCommand(const ros::Time &time) {
  vel_2d_cmd_sender_->setLinearXVel(auto_linear_x_);
  vel_2d_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  lower_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
}

void StateMachine::cruiseChassis() {
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->sendCommand(ros::Time::now());
}

void StateMachine::cruiseGimbal() {
  if (lower_cmd_sender_->pos_yaw_ >= lower_cmd_sender_->yaw_max_)
    lower_cmd_sender_->yaw_direct_ = -1.;
  else if (lower_cmd_sender_->pos_yaw_ <= lower_cmd_sender_->yaw_min_)
    lower_cmd_sender_->yaw_direct_ = 1.;
  if (lower_cmd_sender_->pos_pitch_ >= lower_cmd_sender_->pitch_max_)
    lower_cmd_sender_->pitch_direct_ = -1.;
  else if (lower_cmd_sender_->pos_pitch_ <= lower_cmd_sender_->pitch_min_)
    lower_cmd_sender_->pitch_direct_ = 1.;
  setTrack(lower_cmd_sender_);
}

void StateMachine::cruiseShooter() {
  if (lower_cmd_sender_->gimbal_cmd_sender_->getMsg()->mode ==
      rm_msgs::GimbalCmd::TRACK) {
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    lower_cmd_sender_->shooter_cmd_sender_->checkError(
        lower_cmd_sender_->gimbal_des_error_, ros::Time::now());
  } else
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void StateMachine::rawChassis() {
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->sendCommand(ros::Time::now());
}

void StateMachine::rawGimbal() {
  lower_cmd_sender_->gimbal_cmd_sender_->setRate(-fsm_data_.dbus_data_.ch_l_x,
                                                 -fsm_data_.dbus_data_.ch_l_y);
  if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN)
    lower_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else {
    lower_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(
        lower_cmd_sender_->shooter_cmd_sender_->getSpeed());
    lower_cmd_sender_->gimbal_cmd_sender_->updateCost(
        lower_cmd_sender_->track_data_);
  }
}

void StateMachine::rawShooter() {
  if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::UP) {
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    lower_cmd_sender_->shooter_cmd_sender_->checkError(
        lower_cmd_sender_->gimbal_des_error_, ros::Time::now());
  } else if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::MID)
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  else
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::setTrack(SideCommandSender *side_cmd_sender) {
  side_cmd_sender->gimbal_cmd_sender_->setBulletSpeed(
      side_cmd_sender->shooter_cmd_sender_->getSpeed());
  side_cmd_sender->gimbal_cmd_sender_->updateCost(side_cmd_sender->track_data_);
  if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode ==
      rm_msgs::GimbalCmd::TRACK)
    side_cmd_sender->gimbal_cmd_sender_->setRate(0., 0.);
  else
    side_cmd_sender->gimbal_cmd_sender_->setRate(
        side_cmd_sender->yaw_direct_, side_cmd_sender->pitch_direct_);
}

void StateMachine::checkReferee(const ros::Time &time) {
  if (fsm_data_.game_robot_status_
          .mains_power_chassis_output_ &&
      !chassis_output_) {
    ROS_INFO("Chassis output ON");
    chassisOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_gimbal_output_ &&
      !gimbal_output_) {
    ROS_INFO("Gimbal output ON");
    gimbalOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_shooter_output_ &&
      !shooter_output_) {
    ROS_INFO("Shooter output ON");
    shooterOutputOn();
  }
  if (fsm_data_.game_robot_status_.mains_power_chassis_output_)
    chassis_output_ = true;
  else
    chassis_output_ = false;
  if (fsm_data_.game_robot_status_.mains_power_gimbal_output_)
    gimbal_output_ = true;
  else
    gimbal_output_ = false;
  if (fsm_data_.game_robot_status_.mains_power_shooter_output_)
    shooter_output_ = true;
  else
    shooter_output_ = false;
}

void StateMachine::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - fsm_data_.dbus_data_.stamp).toSec() > 0.3) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - fsm_data_.dbus_data_.stamp).toSec() < 0.3) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
}

void StateMachine::chassisOutputOn() {
  ROS_INFO("Chassis output ON");
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void StateMachine::gimbalOutputOn() {
  ROS_INFO("Gimbal output ON");
  lower_gimbal_calibration_->reset();
}

void StateMachine::shooterOutputOn() {
  ROS_INFO("Shooter output ON");
  lower_trigger_calibration_->reset();
  lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
}

void StateMachine::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
  lower_trigger_calibration_->stopController();
}

void StateMachine::cruiseRun() {
  fsm_data_.update(ros::Time::now());
  checkReferee(ros::Time::now());
  checkSwitch(ros::Time::now());
  update(ros::Time::now());
  cruiseGimbal();
  cruiseShooter();
  sendCruiseCommand(ros::Time::now());
  controller_manager_.update();
}

void StateMachine::rawRun() {
  fsm_data_.update(ros::Time::now());
  checkReferee(ros::Time::now());
  checkSwitch(ros::Time::now());
  rawGimbal();
  rawShooter();
  sendRawCommand(ros::Time::now());
  controller_manager_.update();
}
