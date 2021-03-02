//
// Created by luohx on 7/27/20.
//

#include <ros/ros.h>
#include "rm_fsm/fsm_common.h"
#include "rm_fsm/fsm_sentry.h"
#include "rm_fsm/fsm_standard.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  Fsm<float> *control_fsm;
  if(robot=="standard")
    control_fsm = new FsmStandard<float>(nh);
  else if (robot == "sentry")
    control_fsm = new FsmSentry<float>(nh);
  else {
    ROS_ERROR("no robot type load");
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