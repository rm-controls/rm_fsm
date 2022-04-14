//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/common/fsm_data.h"
#include "rm_fsm/StateMachine.h"

int main(int argc, char **argv) {
    std::string robot;
    ros::init(argc, argv, "rm_fsm");
    ros::NodeHandle fsm_nh("~");
    robot = getParam(fsm_nh, "robot_type", (std::string) "error");
    if (robot == "sentry") {
        ROS_INFO("Running sentry robot.");
    } else {
        ROS_ERROR("No robot type load");
        return 0;
    }
    StateMachine sm(fsm_nh);
    ROS_INFO("Enter sentry robot.");
    std::default_random_engine random;
    std::uniform_real_distribution<double> real_random(1.0, 3.0);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::Time time = ros::Time::now();
        sm.fsm_data_.update(time);
        ros::Time begin_time = ros::Time::now();
        if ((begin_time - sm.last_time_).toSec() == sm.rand_time_ ||
            (begin_time - sm.last_time_).toSec() > sm.rand_time_) {
            ROS_INFO("difference: %f, rand time: %f", (begin_time - sm.last_time_).toSec(), sm.rand_time_);
            sm.changeVel();
            sm.last_time_ = ros::Time::now();
            sm.rand_time_ = real_random(random);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
