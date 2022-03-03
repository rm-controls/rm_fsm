//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/StateMachine.h"

StateMachine::StateMachine(ros::NodeHandle& nh) : context_(*this), controller_manager_(nh)
{
    nh_ = ros::NodeHandle(nh, "StateMachine");
    try {
        XmlRpc::XmlRpcValue lower_trigger_rpc_value, lower_gimbal_rpc_value;
        nh.getParam("lower_trigger_calibration", lower_trigger_rpc_value);
        nh.getParam("lower_gimbal_calibration", lower_gimbal_rpc_value);
        lower_trigger_calibration_ = new rm_common::CalibrationQueue(lower_trigger_rpc_value, nh, controller_manager_);
        lower_gimbal_calibration_ = new rm_common::CalibrationQueue(lower_gimbal_rpc_value, nh, controller_manager_);
    } catch (XmlRpc::XmlRpcException &e) {
        ROS_ERROR("%s", e.getMessage().c_str());
    }
    context_.enterStartState();
}

void StateMachine::initCalibrate()
{
    if (!nh_.getParam("collision_effort", collision_effort_)) {
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

void StateMachine::initStandby()
{
    if (!nh_.getParam("move_distance", move_distance_))
        ROS_ERROR("Move distance no defined (namespace: %s)",nh_.getNamespace().c_str());
    if (!nh_.getParam("stop_distance", stop_distance_))
        ROS_ERROR("Stop distance no defined (namespace: %s)", nh_.getNamespace().c_str());
}

void StateMachine::initCruise()
{}

void StateMachine::initRaw()
{}

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

void StateMachine::run() {
    setChassis();
    setGimbal(upper_cmd_sender_);
    setGimbal(lower_cmd_sender_);
    setShooter(upper_cmd_sender_);
    setShooter(lower_cmd_sender_);
    sendCommand(ros::Time::now());
}

void StateMachine::sendCommand(const ros::Time &time) {
    chassis_cmd_sender_->sendCommand(time);
    vel_2d_cmd_sender_->sendCommand(time);
    upper_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
    lower_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
    upper_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
    lower_cmd_sender_->shooter_cmd_sender_->sendCommand(time);
}