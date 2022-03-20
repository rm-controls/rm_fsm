//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle& nh) : context_(*this), controller_manager_(nh)
{
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
    context_.enterStartState();
}

void StateMachine::initCalibrate() {
    ROS_INFO("Enter Calibrate");
    ros::NodeHandle auto_nh = ros::NodeHandle(nh_, "auto");
    if (!auto_nh.getParam("collision_effort", collision_effort_)) {
        ROS_ERROR("Collision effort no defined (namespace: %s)", nh_.getNamespace().c_str());
    }
    odom2baselink_.header.frame_id = "odom";
    odom2baselink_.child_frame_id = "base_link";
    map2odom_.header.stamp = ros::Time::now();
    map2odom_.header.frame_id = "map";
    map2odom_.child_frame_id = "odom";
    map2odom_.transform.translation.x = 0.;
    map2odom_.transform.translation.y = 0.;
    map2odom_.transform.translation.z = 0.;
    map2odom_.transform.rotation.w = 1.;
    static_tf_broadcaster_.sendTransform(map2odom_);
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
    start_flag_ = false;
    start_pos_ = 0.;
    random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
}

void StateMachine::initRaw() {
    ROS_INFO("Enter Raw");
}

void StateMachine::checkCalibrateStatus()
{
    if (ros::Time::now() - init_time_ > ros::Duration(1.0) && !finish_calibrate_
    && fsm_data_.current_effort_ >= collision_effort_) {
        finish_calibrate_ = true;
        map2odom_.header.stamp = ros::Time::now();
        map2odom_.transform.translation.x = fsm_data_.pos_x_ + 0.15;
        map2odom_.transform.translation.y = 0.;
        map2odom_.transform.translation.z = 0.;
        map2odom_.transform.rotation.w = 1.;
        static_tf_broadcaster_.sendTransform(map2odom_);
        odom2baselink_.header.stamp = ros::Time::now();
        odom2baselink_.transform.translation.x = 0.;
        odom2baselink_.transform.translation.y = 0.;
        odom2baselink_.transform.translation.z = 0.;
        odom2baselink_.transform.rotation.w = 1.;
        tf_broadcaster_.sendTransform(odom2baselink_);
    }
}

void StateMachine::sendRawCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(dbus_.ch_r_y);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::sendCalibrateCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(1.);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::sendCruiseCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    if (vel_2d_cmd_sender_->getMsg()->linear.x == 0.) vel_2d_cmd_sender_->setLinearXVel(1.);
    if (fsm_data_.pos_x_ <= start_pos_ - random_distance_ || fsm_data_.pos_x_ <= -move_distance_) {
        vel_2d_cmd_sender_->setLinearXVel(1.);
        if (!start_flag_) {
            start_pos_ = fsm_data_.pos_x_;
            random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
            start_flag_ = true;
        }
    } else if (fsm_data_.pos_x_ >= start_pos_ + random_distance_ || fsm_data_.pos_x_ >= 0.) {
        vel_2d_cmd_sender_->setLinearXVel(-1.);
        if (!start_flag_) {
            start_pos_ = fsm_data_.pos_x_;
            random_distance_ = move_distance_ * 0.3 * ((double) rand() / RAND_MAX);
            start_flag_ = true;
        }
    } else start_flag_ = false;
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::sendStandbyCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->setLinearXVel(0.);
    vel_2d_cmd_sender_->sendCommand(time);
}

void StateMachine::cruiseChassis() {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void StateMachine::cruiseGimbal() {
    lower_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void StateMachine::cruiseShooter() {
    lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
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

void StateMachine::standbyChassis() {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void StateMachine::standbyGimbal() {
    if (lower_cmd_sender_->pos_yaw_ >= lower_cmd_sender_->yaw_max_) lower_cmd_sender_->yaw_direct_ = -1.;
    else if (lower_cmd_sender_->pos_yaw_ <= lower_cmd_sender_->yaw_min_) lower_cmd_sender_->yaw_direct_ = 1.;
    if (lower_cmd_sender_->pos_pitch_ >= lower_cmd_sender_->pitch_max_) lower_cmd_sender_->pitch_direct_ = -1.;
    else if (lower_cmd_sender_->pos_pitch_ <= lower_cmd_sender_->pitch_min_) lower_cmd_sender_->pitch_direct_ = 1.;
    setTrack(lower_cmd_sender_);
}

void StateMachine::setTrack(SideCommandSender *side_cmd_sender) {
    side_cmd_sender->gimbal_cmd_sender_->setBulletSpeed(side_cmd_sender->shooter_cmd_sender_->getSpeed());
    side_cmd_sender->gimbal_cmd_sender_->updateCost(side_cmd_sender->track_data_);
    if (side_cmd_sender->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK)
        side_cmd_sender->gimbal_cmd_sender_->setRate(0., 0.);
    else
        side_cmd_sender->gimbal_cmd_sender_->setRate(side_cmd_sender->yaw_direct_, side_cmd_sender->pitch_direct_);
}

void StateMachine::standbyShooter() {
    if (lower_cmd_sender_->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK) {
        lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        lower_cmd_sender_->shooter_cmd_sender_->checkError(lower_cmd_sender_->gimbal_des_error_, ros::Time::now());
    } else lower_cmd_sender_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void StateMachine::calibrateChassis() {
    ROS_INFO("Set Chassis");
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    ROS_INFO("finish");
}