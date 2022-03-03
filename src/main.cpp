//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/common/fsm_data.h"
#include "rm_fsm/StateMachine.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle fsm_nh("~");
  StateMachine sm(fsm_nh);
  robot = getParam(fsm_nh, "robot_type", (std::string) "error");
  if (robot == "sentry") {
    ROS_INFO("Running sentry robot.");
  } else {
    ROS_ERROR("No robot type load");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::Time time = ros::Time::now();
    sm.fsm_data_.update(time);
    ros::spinOnce();
    loop_rate.sleep();
    try {
        if (sm.fsm_data_.serial_.available()) {
            sm.fsm_data_.referee_.rx_len_ = (int) sm.fsm_data_.serial_.available();
            sm.fsm_data_.serial_.read(sm.fsm_data_.referee_.rx_buffer_, sm.fsm_data_.referee_.rx_len_);
        }
    }
    catch (serial::IOException &e) {}
    sm.fsm_data_.referee_.read();
  }
  return 0;
}
