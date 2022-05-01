//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle &nh) : context_(*this), controller_manager_(nh) {
    nh_ = ros::NodeHandle(nh);
    try {
        XmlRpc::XmlRpcValue lower_trigger_rpc_value, lower_gimbal_rpc_value;
        nh.getParam("lower_trigger_calibration", lower_trigger_rpc_value);
        nh.getParam("lower_gimbal_calibration", lower_gimbal_rpc_value);
        lower_trigger_calibration_ = new rm_common::CalibrationQueue(lower_trigger_rpc_value, nh, controller_manager_);
        lower_gimbal_calibration_ = new rm_common::CalibrationQueue(lower_gimbal_rpc_value, nh, controller_manager_);
    } catch (XmlRpc::XmlRpcException &e) {
        ROS_ERROR("%s", e.getMessage().c_str());
    }
    ros::NodeHandle chassis_nh(nh_, "chassis");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, fsm_data_.referee_.referee_data_);
    ros::NodeHandle vel_nh(nh_, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &StateMachine::dbusCB, this);
    referee_sub_ = nh_.subscribe<rm_msgs::Referee>("/referee", 10, &StateMachine::refereeCB, this);
    radar_sub_ = nh_.subscribe<rm_msgs::TfRadarData>("/controllers/tf_radar_controller/chassis_radar/data",
                                                     10, &StateMachine::radarCB, this);

    context_.enterStartState();
}


void StateMachine::initStandby() {
    ROS_INFO("Enter Standby");
    ros::NodeHandle auto_nh = ros::NodeHandle(nh_, "auto");
    if (!auto_nh.getParam("move_distance", move_distance_))
        ROS_ERROR("Move distance no defined (namespace: %s)", nh_.getNamespace().c_str());
    if (!auto_nh.getParam("stop_distance", stop_distance_))
        ROS_ERROR("Stop distance no defined (namespace: %s)", nh_.getNamespace().c_str());
}

void StateMachine::initCruise() {
    ROS_INFO("Enter Cruise");
    ros::NodeHandle auto_nh(nh_, "auto");
    if (!auto_nh.getParam("auto_linear_x", auto_linear_x_)) {
        ROS_ERROR("Can not find auto_linear_x");
    }
    start_flag_ = false;
    start_pos_ = 0.;
}

void StateMachine::initRaw() {
    ROS_INFO("Enter Raw");
}

void StateMachine::sendRawCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(dbus_.ch_r_y);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::sendCruiseCommand(const ros::Time &time) {
    vel_2d_cmd_sender_->setLinearXVel(auto_linear_x_);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::sendStandbyCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(0.);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::cruiseChassis() {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    chassis_cmd_sender_->sendCommand(ros::Time::now());
}

void StateMachine::cruiseGimbal() {
    lower_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    if (lower_cmd_sender_->pos_yaw_ >= lower_cmd_sender_->yaw_max_) lower_cmd_sender_->yaw_direct_ = -1.;
    else if (lower_cmd_sender_->pos_yaw_ <= lower_cmd_sender_->yaw_min_) lower_cmd_sender_->yaw_direct_ = 1.;
    if (lower_cmd_sender_->pos_pitch_ >= lower_cmd_sender_->pitch_max_) lower_cmd_sender_->pitch_direct_ = -1.;
    else if (lower_cmd_sender_->pos_pitch_ <= lower_cmd_sender_->pitch_min_) lower_cmd_sender_->pitch_direct_ = 1.;
    setTrack(lower_cmd_sender_);
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::cruiseShooter() {
    if (lower_cmd_sender_->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
        lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        lower_cmd_sender_->shooter_cmd_sender_->checkError(lower_cmd_sender_->gimbal_des_error_, ros::Time::now());
    } else lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void StateMachine::rawChassis() {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void StateMachine::rawGimbal() {
    lower_cmd_sender_->gimbal_cmd_sender_->setRate(-fsm_data_.dbus_data_.ch_l_x, -fsm_data_.dbus_data_.ch_l_y);
    if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN)
        lower_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    else {
        lower_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(lower_cmd_sender_->shooter_cmd_sender_->getSpeed());
        lower_cmd_sender_->gimbal_cmd_sender_->updateCost(lower_cmd_sender_->track_data_);
    }
}

void StateMachine::rawShooter() {
    if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::UP) {
        lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        lower_cmd_sender_->shooter_cmd_sender_->checkError(lower_cmd_sender_->gimbal_des_error_, ros::Time::now());
    } else if (fsm_data_.dbus_data_.s_l == rm_msgs::DbusData::MID)
        lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    else lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void StateMachine::setTrack(SideCommandSender *side_cmd_sender) {
    side_cmd_sender->gimbal_cmd_sender_->setBulletSpeed(side_cmd_sender->shooter_cmd_sender_->getSpeed());
    side_cmd_sender->gimbal_cmd_sender_->updateCost(side_cmd_sender->track_data_);
    if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK)
        side_cmd_sender->gimbal_cmd_sender_->setRate(0., 0.);
    else
        side_cmd_sender->gimbal_cmd_sender_->setRate(side_cmd_sender->yaw_direct_, side_cmd_sender->pitch_direct_);
}
