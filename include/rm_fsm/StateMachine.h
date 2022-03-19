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
    StateMachine(ros::NodeHandle &nh);

    bool isCalibrate(rm_msgs::DbusData data_dbus_) {
        if (data_dbus_.s_r == rm_msgs::DbusData::UP)
            return true;
        else
            return false;
    }

    bool isRaw(rm_msgs::DbusData data_dbus_) {
        if (data_dbus_.s_r == rm_msgs::DbusData::MID)
            return true;
        else
            return false;
    }

    bool isStandby(rm_msgs::Referee referee_) {
        if (referee_.data_cmd_id == 0x0200
            && referee_.interactive_data == 0 && rm_msgs::DbusData::UP)
            return true;
        else
            return false;
    }

    bool isCruise(rm_msgs::Referee referee_) {
        if (!(referee_.data_cmd_id == 0x0200
              && referee_.interactive_data == 0 && dbus_.s_r == rm_msgs::DbusData::UP))
            return true;
        else
            return false;
    }

    void initRaw();

    void initCalibrate();

    void checkCalibrateStatus();

    bool getCalibrateStatus() const { return finish_calibrate_; }

    void initStandby();

    void initCruise();

    void sendCommand(const ros::Time &time);

    void cruiseChassis();

    void cruiseShooter();

    void cruiseGimbal();

    void standbyChassis();

    void standbyGimbal();

    void setTrack(SideCommandSender *side_cmd_sender);

    void standbyShooter();

    void rawChassis();

    void rawGimbal();

    void rawShooter();

    void run();

    FsmData fsm_data_;

protected:
    StateMachineContext context_;
    rm_msgs::DbusData dbus_;

    rm_common::CalibrationQueue *lower_trigger_calibration_{}, *lower_gimbal_calibration_{};
    rm_common::ControllerManager controller_manager_;
    ros::NodeHandle nh_;
    //calibrate
    double collision_effort_{};
    bool finish_calibrate_ = false, init_flag_ = false;
    double move_distance_{}, stop_distance_{};
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped map2odom_, odom2baselink_;
    ros::Time init_time_ = ros::Time::now();
    rm_common::ChassisCommandSender *chassis_cmd_sender_;
    rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
    SideCommandSender *upper_cmd_sender_, *lower_cmd_sender_;
    //cruise
    double random_distance_, start_pos_;
    bool start_flag_;
    ros::Time last_send_ = ros::Time::now();

    ros::Subscriber dbus_sub_;
    ros::Subscriber referee_sub_;

    void dbusCB(const rm_msgs::DbusData::ConstPtr &dbus_data) {
        dbus_ = *dbus_data;
        context_.dbusUpdate(*dbus_data);
    }

    void refereeCB(const rm_msgs::Referee::ConstPtr &referee_data) {
        context_.refereeUpdate(*referee_data);
    }

};
