//
// Created by luohx on 7/27/20.
//

#include <ros/ros.h>
#include <rm_fsm/fsm_common.h>
#include "rm_fsm/fsm_sentry.h"
#include "rm_fsm/fsm_standard.h"
int main(int argc, char **argv) {
  std::string robot_type;
  ros::init(argc, argv, "fsm");
  ros::NodeHandle nh("fsm");
  Fsm<float>* robot_fsm;
  robot_type = getParam(nh,"robot_type",std::string("error"));
  if(robot_type=="sentry")
    robot_fsm = new FsmSentry<float>(nh);
  else if(robot_type == "standard")
    robot_fsm = new FsmStandard<float>(nh);
  else
  {
    ROS_ERROR("can not load robot type");
    return 1;
  }
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    robot_fsm->run();
  }
  return 0;
}