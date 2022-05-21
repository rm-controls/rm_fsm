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
        lower_trigger_calibration_ = new rm_common::CalibrationQueue(lower_trigger_rpc_value, nh_, controller_manager_);
        lower_gimbal_calibration_ = new rm_common::CalibrationQueue(lower_gimbal_rpc_value, nh_, controller_manager_);
    } catch (XmlRpc::XmlRpcException &e) {
        ROS_ERROR("%s", e.getMessage().c_str());
    }
    ros::NodeHandle chassis_nh(nh_, "chassis");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, fsm_data_.referee_.referee_data_);
    ros::NodeHandle vel_nh(nh_, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    ros::NodeHandle lower_nh(nh_, "lower");
    lower_cmd_sender_ = new SideCommandSender(lower_nh, fsm_data_.referee_.referee_data_,
                                              fsm_data_.lower_track_data_array_,
                                              fsm_data_.lower_gimbal_des_error_, fsm_data_.lower_yaw_,
                                              fsm_data_.lower_pitch_);
    dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &StateMachine::dbusCB, this);
    referee_sub_ = nh_.subscribe<rm_msgs::Referee>("/referee", 10, &StateMachine::refereeCB, this);
    left_radar_sub_ = nh_.subscribe<rm_msgs::TfRadarData>("/controllers/tf_radar_controller/left_tf_radar/data",
                                                          10, &StateMachine::leftRadarCB, this);
    right_radar_sub_ = nh_.subscribe<rm_msgs::TfRadarData>("/controllers/tf_radar_controller/right_tf_radar/data",
                                                           10, &StateMachine::rightRadarCB, this);

    context_.enterStartState();
}

void StateMachine::update(const ros::Time &time) {
    lower_cmd_sender_->pos_pitch_ = fsm_data_.lower_pitch_;
    lower_cmd_sender_->pos_yaw_ = fsm_data_.lower_yaw_;
    lower_cmd_sender_->track_data_ = fsm_data_.lower_track_data_array_;
    lower_cmd_sender_->gimbal_des_error_ = fsm_data_.lower_gimbal_des_error_;
    fsm_data_.updatePosX(time);
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

void StateMachine::initCruise() {
    ROS_INFO("Enter Cruise");
    ros::NodeHandle auto_nh(nh_, "auto");
    if (!auto_nh.getParam("auto_linear_x", auto_linear_x_)) {
        ROS_ERROR("Can not find auto_linear_x");
    }
}

void StateMachine::sendRawCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(dbus_.ch_r_y);
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
    if (lower_cmd_sender_->pos_yaw_ >= lower_cmd_sender_->yaw_max_) lower_cmd_sender_->yaw_direct_ = -1.;
    else if (lower_cmd_sender_->pos_yaw_ <= lower_cmd_sender_->yaw_min_) lower_cmd_sender_->yaw_direct_ = 1.;
    if (lower_cmd_sender_->pos_pitch_ >= lower_cmd_sender_->pitch_max_) lower_cmd_sender_->pitch_direct_ = -1.;
    else if (lower_cmd_sender_->pos_pitch_ <= lower_cmd_sender_->pitch_min_) lower_cmd_sender_->pitch_direct_ = 1.;
    setTrack(lower_cmd_sender_);
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
