//
// Created by astro on 2021/1/26.
//

#ifndef RM_FSM_FSM_SENTRY_H_
#define RM_FSM_FSM_SENTRY_H_

#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/state_raw.h"
#include "rm_fsm/state_calibrate.h"
#include "rm_fsm/state_cruise.h"
#include "rm_fsm/state_standby.h"

namespace rm_fsm {
class FsmSentry : public FsmBase {
 public:
  explicit FsmSentry(ros::NodeHandle &nh) : FsmBase(nh) {
    string2state_.insert(std::pair<std::string, StateBase *>(state_raw_->getName(), state_raw_));
    string2state_.insert(std::pair<std::string, StateBase *>(state_calibrate_->getName(), state_calibrate_));
    string2state_.insert(std::pair<std::string, StateBase *>(state_standby_->getName(), state_standby_));
    string2state_.insert(std::pair<std::string, StateBase *>(state_cruise_->getName(), state_cruise_));
    string2state_.insert(std::pair<std::string, StateBase *>(state_idle_->getName(), state_idle_));
    current_state_ = string2state_["RAW"];
    try {
      XmlRpc::XmlRpcValue upper_trigger_rpc_value, upper_gimbal_rpc_value, lower_trigger_rpc_value,
          lower_gimbal_rpc_value;
      nh.getParam("upper_trigger_calibration", upper_trigger_rpc_value);
      nh.getParam("upper_gimbal_calibration", upper_gimbal_rpc_value);
      nh.getParam("lower_trigger_calibration", lower_trigger_rpc_value);
      nh.getParam("lower_gimbal_calibration", lower_gimbal_rpc_value);
      upper_trigger_calibration_ = new rm_common::CalibrationQueue(upper_trigger_rpc_value, nh, controller_manager_);
      upper_gimbal_calibration_ = new rm_common::CalibrationQueue(upper_gimbal_rpc_value, nh, controller_manager_);
      lower_trigger_calibration_ = new rm_common::CalibrationQueue(lower_trigger_rpc_value, nh, controller_manager_);
      lower_gimbal_calibration_ = new rm_common::CalibrationQueue(lower_gimbal_rpc_value, nh, controller_manager_);
    } catch (XmlRpc::XmlRpcException &e) {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
  }
  void run() override {
    FsmBase::run();
    upper_trigger_calibration_->update(ros::Time::now());
    upper_gimbal_calibration_->update(ros::Time::now());
    lower_trigger_calibration_->update(ros::Time::now());
    lower_gimbal_calibration_->update(ros::Time::now());
  }
 protected:
  std::string getNextState() override {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
      if (!state_calibrate_->getCalibrateStatus()) return "CALIBRATE";
      sendMode(ros::Time::now());
      if (data_.referee_.referee_data_.interactive_data.header_data_.data_cmd_id_ == 0x0200
          && data_.referee_.referee_data_.interactive_data.data_ == 0)
        return "STANDBY";
      else return "CRUISE";
    } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) return "RAW";
    else return "IDLE";
  }

  void remoteControlTurnOn() override {
    FsmBase::remoteControlTurnOn();
    upper_trigger_calibration_->reset();
    upper_gimbal_calibration_->reset();
    lower_trigger_calibration_->reset();
    lower_gimbal_calibration_->reset();
  }
 private:
  void sendMode(const ros::Time &time) {
    if (time - last_send_ < ros::Duration(0.5)) return;
    int receiver_id = data_.referee_.referee_data_.robot_id_ == rm_common::RED_SENTRY ? rm_common::RED_ENGINEER
                                                                                      : rm_common::BLUE_ENGINEER;
    data_.referee_.sendInteractiveData(0x0201, receiver_id, data_.referee_.referee_data_.interactive_data.data_);
    last_send_ = time;
  }
  StateBase *state_idle_ = new StateBase(nh_, &data_, "IDLE");
  StateRaw *state_raw_ = new StateRaw(nh_, &data_);
  StateCalibrate *state_calibrate_ = new StateCalibrate(nh_, &data_);
  StateStandby *state_standby_ = new StateStandby(nh_, &data_);
  StateCruise *state_cruise_ = new StateCruise(nh_, &data_);
  ros::Time last_send_ = ros::Time::now();
};
}

#endif //RM_FSM_FSM_SENTRY_H_
