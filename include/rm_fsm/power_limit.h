//
// Created by kiana on 2021/1/26.
//

#ifndef SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
#define SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include "rm_fsm/referee.h"
#include <rm_msgs/PowerLimit.h>
#include <rm_common/filters/lp_filter.h>
#include <rm_common/filters/filters.h>
#include <sensor_msgs/JointState.h>
#include <rm_fsm/PowerLimitConfig.h>
#include <math.h>

class PowerLimit {
 public:
  explicit PowerLimit(ros::NodeHandle &nh);
  void input(RefereeData referee_data_,
             PowerManagerData power_manager_data_,
             bool use_power_manager);
  double output();
  double getLimitPower(RefereeData referee_data_);
  double getSafetyEffort();
  void getLimitEffort();

 protected:
  virtual void reconfigCB(rm_fsm::PowerLimitConfig &config, uint32_t /*level*/);

 private:
  ros::Time last_run_;
  ros::Subscriber joint_state_sub_;
  control_toolbox::Pid pid_buffer_;
  control_toolbox::Pid pid_buffer_power_manager_;
  void jointVelCB(const sensor_msgs::JointState &data);

  double real_chassis_power_;
  double limit_power_;
  double capacity_;
  double error_power_;

  double safety_effort_;
  double wheel_radius_;
  double ff_;
  double max_limit_;
  double max_limit_50w_;
  double max_limit_60w_;
  double max_limit_70w_;
  double max_limit_80w_;
  double max_limit_100w_;
  double max_limit_120w_;

  bool have_capacity_;
  double pid_counter_;
  double vel_total;
  double last_vel_total_;

  double power_offset_;
  LowPassFilter *lp_error_{};
  LowPassFilter *lp_real_power_{};
  RampFilter<double> *ramp_error_{};
  //publish some data for test
  ros::Publisher power_limit_pub_;
  rm_msgs::PowerLimit power_limit_pub_data_;
  dynamic_reconfigure::Server<rm_fsm::PowerLimitConfig> *d_srv_{};

};

#endif //SRC_RM_FSM_INCLUDE_RM_FSM_POWER_LIMIT_H_
