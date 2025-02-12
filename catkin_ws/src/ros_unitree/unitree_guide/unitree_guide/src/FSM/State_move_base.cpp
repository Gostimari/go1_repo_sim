/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
    ros::spinOnce();
}

/*
void State_move_base::twistCallbackz(const geometry_msgs::Twist& msg){
    _wz = msg.angular.z;
}*/

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}

/*
void State_move_base::twistCallback(const nav_msgs::Path::ConstPtr& msg)
{
    for (i = 0; i < msg->poses.size(); ++i)
        _vx = msg->poses[0].pose.position.x;
        _vy = msg->poses[0].pose.position.y;
        _wz = msg->poses[0].pose.orientation.z;
}*/

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
    //_cmdSubz = _nm.subscribe("/wz", 1, &State_move_base::twistCallbackz, this);
}

#endif  // COMPILE_WITH_MOVE_BASE