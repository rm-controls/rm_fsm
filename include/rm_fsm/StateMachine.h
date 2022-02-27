//
// Created by luotinkai on 2022/2/27.
//

#ifndef SRC_STATEMACHINE_H
#define SRC_STATEMACHINE_H

#endif //SRC_STATEMACHINE_H

#include "rm_fsm/common/data.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_msgs/FsmCmd.h>
#include <realtime_tools/realtime_buffer.h>

class StateMachine {
public:
    StateMachine(ros::NodeHandle &nh, std::string state_name);
    bool isCalibrate()
    {
        if (command_.state == rm_msgs::FsmCmd::CALIBRATE)
            return true;
        else
            return false;
    }
    bool isIdle()
    {
        if (command_.state == rm_msgs::FsmCmd::IDLE)
            return true;
        else
            return false;
    }
    bool isRaw()
    {
        if (command_.state == rm_msgs::FsmCmd::RAW)
            return true;
        else
            return false;
    }
    bool isStandby()
    {
        if (command_.state == rm_msgs::FsmCmd::STANDBY)
            return true;
        else
            return false;
    }
    bool isCruise()
    {
        if (command_.state == rm_msgs::FsmCmd::CRUISE)
            return true;
        else
            return false;
    }
    void commandCB(const rm_msgs::FsmCmdConstPtr& msg)
    {
        cmd_rt_buffer_.writeFromNonRT(*msg);
        command_.state = cmd_rt_buffer_.readFromRT()->state;
    }

protected:
    rm_msgs::FsmCmd command_;
    realtime_tools::RealtimeBuffer<rm_msgs::FsmCmd> cmd_rt_buffer_;

};
