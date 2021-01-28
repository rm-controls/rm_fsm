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

template<typename T>
class FsmData {
 public:
  FsmData() = default;

  ros::Subscriber dbus_sub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber robot_status_sub_;
  ros::Subscriber game_status_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber euler_sub_;

  rm_msgs::DbusData dbus_data_;
  rm_msgs::Joint joint_data_;
  sensor_msgs::Imu imu_data_;
  geometry_msgs::Vector3 euler_;

  //chassis
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel;
  double max_chassis_speed_[3]{};
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  PowerLimit *power_limit_{};

  //gimbal
  rm_msgs::GimbalCmd gimbal_cmd_;
  ros::Publisher gimbal_cmd_pub_;
  double max_gimbal_rate_[2]{};

  //shooter
  rm_msgs::ShootCmd shoot_cmd_;
  ros::Publisher shooter_cmd_pub_;

  referee::Referee *referee_{};

  void init(ros::NodeHandle nh) {
    power_limit_ = new PowerLimit(nh);
    referee_ = new referee::Referee();
    referee_->init();
    // sub //
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
        "/dbus_data", 10, &FsmData::dbusDataCallback, this);
    joint_sub_ = nh.subscribe<rm_msgs::Joint>(
        "/rm_base/joint_data", 10, &FsmData::jointDataCallback, this);
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(
        "/rm_base/imu_data", 10, &FsmData::imuDataCallback, this);
    euler_sub_ = nh.subscribe<geometry_msgs::Vector3>(
        "/rm_base/euler", 10, &FsmData::eulerCallback, this);
    // pub //
    ros::NodeHandle root_nh;
    vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
    gimbal_cmd_pub_ = root_nh.advertise<rm_msgs::GimbalCmd>("/cmd_gimbal", 1);
    shooter_cmd_pub_ = root_nh.advertise<rm_msgs::ShootCmd>("/cmd_shoot", 1);
    referee_->referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
  }

  void jointDataCallback(const rm_msgs::Joint::ConstPtr &data) {
    joint_data_ = *data;
  }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }
  void imuDataCallback(const sensor_msgs::ImuConstPtr &data) {
    imu_data_ = *data;
  }
  void eulerCallback(const geometry_msgs::Vector3::ConstPtr &data) {
    euler_ = *data;
  }

};

#endif //RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
