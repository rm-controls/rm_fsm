//
// Created by luotinkai on 2022/3/2.
//

#include "rm_fsm/common/fsm_data.h"

FsmData::FsmData()
{
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &FsmData::jointStateCallback, this);
    dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &FsmData::dbusDataCallback, this);
    lower_track_sub_ =
            nh_.subscribe<rm_msgs::TrackDataArray>("/controllers/lower_gimbal_controller/track", 10,
                                                  &FsmData::lowerTrackCallback, this);
    lower_gimbal_des_error_sub_ =
            nh_.subscribe<rm_msgs::GimbalDesError>("/controllers/lower_gimbal_controller/error_des", 10,
                                                  &FsmData::lowerGimbalDesErrorCallback, this);
    initSerial();
}

void FsmData::initSerial() {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen()) return;
    try {
        serial_.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR("Cannot open referee port");
    }
}

void FsmData::update(const ros::Time &time) {
    geometry_msgs::TransformStamped odom2baselink;
    if (!joint_state_.effort.empty())
        sum_effort_ += joint_state_.effort[0];
    sum_count_++;
    if (time - update_effort_ > ros::Duration(0.1)) {
        if (sum_count_ != 0) current_effort_ = sum_effort_ / sum_count_;
        sum_effort_ = 0.;
        sum_count_ = 0;
    }
    try { odom2baselink = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_ERROR_ONCE("%s", ex.what()); }
    pos_x_ = odom2baselink.transform.translation.x;
    if (!joint_state_.position.empty()) {
        upper_yaw_ = joint_state_.position[10];
        upper_pitch_ = joint_state_.position[7];
        lower_yaw_ = joint_state_.position[5];
        lower_pitch_ = joint_state_.position[2];
    }
}
