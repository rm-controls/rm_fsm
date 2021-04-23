//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/fsm_common.h"
#include "rm_fsm/fsm_sentry.h"
#include "rm_fsm/fsm_standard.h"
#include "rm_fsm/fsm_hero.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  Fsm<float> *control_fsm;
  if (robot == "standard") {
    control_fsm = new FsmStandard<float>(nh);
    ROS_INFO("Running standard robot.");
  } else if (robot == "sentry") {
    control_fsm = new FsmSentry<float>(nh);
    ROS_INFO("Running sentry robot.");
  } else if (robot == "hero") {
    control_fsm = new FsmHero<float>(nh);
    ROS_INFO("Running hero robot.");
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