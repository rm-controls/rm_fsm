//
// Created by luohx on 7/27/20.
//

#include <ros/ros.h>
#include <rm_fsm/fsm_common.h>
#include <rm_fsm/fsm_standard.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fsm");
  ros::NodeHandle nh;
  FsmStandard<float> control_fsm(nh);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    control_fsm.run();
  }
  return 0;
}