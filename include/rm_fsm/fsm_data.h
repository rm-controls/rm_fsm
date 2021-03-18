//
// Created by luohx on 7/20/20.
//

#ifndef RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
#define RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include "rm_msgs/DbusData.h"
#include "rm_msgs/ChassisCmd.h"
#include "rm_msgs/GimbalCmd.h"
#include "rm_msgs/ShootCmd.h"
#include "rm_msgs/Joint.h"
#include "rm_fsm/referee.h"
#include "rm_fsm/power_limit.h"
#include "rm_fsm/shooter_heat_limit.h"

template<typename T>
class FsmData {
 public:
  FsmData() = default;

  ros::Subscriber dbus_sub_;

  rm_msgs::DbusData dbus_data_;

  //chassis
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  PowerLimit *power_limit_{};

  //gimbal
  rm_msgs::GimbalCmd gimbal_cmd_;
  ros::Publisher gimbal_cmd_pub_;

  //shooter
  rm_msgs::ShootCmd shoot_cmd_;
  ros::Publisher shooter_cmd_pub_;
  ShooterHeatLimit *shooter_heat_limit_{};

  referee::Referee *referee_{};

  void init(ros::NodeHandle nh) {
    power_limit_ = new PowerLimit(nh);
    shooter_heat_limit_ = new ShooterHeatLimit(nh);
    referee_ = new referee::Referee();

    referee_->init();
    // sub
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
        "/dbus_data", 10, &FsmData::dbusDataCallback, this);
    // pub
    ros::NodeHandle root_nh;
    vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
    gimbal_cmd_pub_ = root_nh.advertise<rm_msgs::GimbalCmd>("/cmd_gimbal", 1);
    shooter_cmd_pub_ = root_nh.advertise<rm_msgs::ShootCmd>("/cmd_shoot", 1);
    referee_->referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
  }

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }
};

#endif //RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
