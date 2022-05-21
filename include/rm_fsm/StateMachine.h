//
// Created by luotinkai on 2022/2/27.
//

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/common/fsm_data.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class StateMachine {
public:
    explicit StateMachine(ros::NodeHandle &nh);

    static bool isRaw(rm_msgs::DbusData data_dbus_) {
        if (data_dbus_.s_r == rm_msgs::DbusData::MID)
            return true;
        else
            return false;
    }

    static bool isCruise(rm_msgs::DbusData data_dbus_) {
        if (data_dbus_.s_r == rm_msgs::DbusData::UP)
            return true;
        else
            return false;
    }

    void changeVel() {
        auto_linear_x_ *= -1.;
    }

    void initRaw();

    void update(const ros::Time &time);

    void initCruise();

    void sendRawCommand(const ros::Time &time);

    void sendCruiseCommand(const ros::Time &time);

    void cruiseChassis();

    void cruiseShooter();

    void cruiseGimbal();

    void setTrack(SideCommandSender *side_cmd_sender);

    void rawChassis();

    void rawGimbal();

    void rawShooter();

    FsmData fsm_data_;
    ros::Time last_time_ = ros::Time::now();
    double rand_time_ = 0.8;

protected:
    StateMachineContext context_;
    rm_msgs::DbusData dbus_;

    rm_common::CalibrationQueue *lower_trigger_calibration_{}, *lower_gimbal_calibration_{};
    rm_common::ControllerManager controller_manager_;
    ros::NodeHandle nh_;
    //calibrate
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Time init_time_ = ros::Time::now();
    rm_common::ChassisCommandSender *chassis_cmd_sender_;
    rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
    SideCommandSender *lower_cmd_sender_;
    //cruise
    ros::Time last_send_ = ros::Time::now();
    double auto_linear_x_{};
    std::default_random_engine random_;
    std::uniform_real_distribution<double> generator_{0.5, 2.5};

    ros::Subscriber dbus_sub_;
    ros::Subscriber referee_sub_;
    ros::Subscriber left_radar_sub_;
    ros::Subscriber right_radar_sub_;

    void dbusCB(const rm_msgs::DbusData::ConstPtr &dbus_data) {
        dbus_ = *dbus_data;
        context_.dbusUpdate(*dbus_data);
    }

    void refereeCB(const rm_msgs::Referee::ConstPtr &referee_data) {
    }

    void leftRadarCB(const rm_msgs::TfRadarDataConstPtr &radar_data) {
        if (radar_data->distance < 0.8 && vel_2d_cmd_sender_->getMsg()->linear.x > 0) {
            last_time_ = ros::Time::now();
            rand_time_ = generator_(random_);
            context_.radarUpdate();
        }
    }

    void rightRadarCB(const rm_msgs::TfRadarDataConstPtr &radar_data) {
        if (radar_data->distance < 0.8 && vel_2d_cmd_sender_->getMsg()->linear.x < 0) {
            last_time_ = ros::Time::now();
            rand_time_ = generator_(random_);
            context_.radarUpdate();
        }
    }

};
