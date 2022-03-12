//
// Created by luotinkai on 2022/3/2.
//

#ifndef SRC_REFEREE_DATA_H
#define SRC_REFEREE_DATA_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <rm_referee/common/referee.h>
#include <tf2_ros/transform_listener.h>
#include <rm_common/decision/command_sender.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackDataArray.h>

class FsmData
        {
        private:
            void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state) { joint_state_ = *joint_state; }
            void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }

            ros::Subscriber joint_state_sub_;
            ros::Subscriber dbus_sub_;
            ros::Subscriber lower_track_sub_;
    ros::NodeHandle nh_;
    double sum_effort_ = 0.;
    int sum_count_ = 0;
    ros::Time update_effort_;

public:
    FsmData();

    ~FsmData() = default;

    void update(const ros::Time &time);

    void lowerTrackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { lower_track_data_array_ = *data; }

    void lowerGimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { lower_gimbal_des_error_ = *data; }

    sensor_msgs::JointState joint_state_;
    rm_msgs::DbusData dbus_data_;
    ros::Subscriber lower_gimbal_des_error_sub_;
    double pos_x_{};
    double upper_yaw_{}, upper_pitch_{}, lower_yaw_{}, lower_pitch_{};
    double current_effort_{};
    tf2_ros::Buffer tf_buffer_;
    rm_msgs::TrackDataArray lower_track_data_array_;
    rm_msgs::GimbalDesError lower_gimbal_des_error_;
    rm_referee::Referee referee_;

};

class SideCommandSender {
public:
    SideCommandSender(ros::NodeHandle &nh, rm_common::RefereeData &referee_data, rm_msgs::TrackDataArray &track_data,
                      rm_msgs::GimbalDesError &gimbal_des_error, double &pos_yaw, double &pos_pitch)
                      : track_data_(track_data), gimbal_des_error_(gimbal_des_error), pos_yaw_(pos_yaw), pos_pitch_(pos_pitch) {
        ros::NodeHandle gimbal_nh(nh, "gimbal");
        gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, referee_data);
        ros::NodeHandle shooter_nh(nh, "shooter");
        shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, referee_data);
        ros::NodeHandle auto_nh(nh, "auto");
        XmlRpc::XmlRpcValue pitch_value, yaw_value;
        try {
            auto_nh.getParam("pitch", pitch_value);
            pitch_min_ = (double) (pitch_value[0]);
            pitch_max_ = (double) (pitch_value[1]);
            auto_nh.getParam("yaw", yaw_value);
            yaw_min_ = (double) (yaw_value[0]);
            yaw_max_ = (double) (yaw_value[1]);
        } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("%s", e.getMessage().c_str()); }
    };
    rm_common::GimbalCommandSender *gimbal_cmd_sender_;
    rm_common::ShooterCommandSender *shooter_cmd_sender_;
    rm_msgs::TrackDataArray &track_data_;
    rm_msgs::GimbalDesError &gimbal_des_error_;
    double pitch_min_{}, pitch_max_{}, yaw_min_{}, yaw_max_{};
    double &pos_yaw_, &pos_pitch_;
    double yaw_direct_{1.}, pitch_direct_{1.};
};

#endif //SRC_REFEREE_DATA_H
