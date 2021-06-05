//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/common/fsm_common.h"
#include "rm_fsm/fsm_sentry.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle fsm_nh("~");
  robot = getParam(fsm_nh, "robot_type", (std::string) "error");
  rm_fsm::FsmBase *control_fsm;
  if (robot == "sentry") {
    control_fsm = new rm_fsm::FsmSentry(fsm_nh);
    ROS_INFO("Running sentry robot.");
  } else {
    ROS_ERROR("No robot type load");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    control_fsm->run();
  }
  return 0;
}