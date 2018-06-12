//
// Created by thomas on 11.06.18.
//
#pragma once

#include <ros/ros.h>
#include "KollrobotMoveGroup.h"
#include <kollrobot_controller/PickBoxAction.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<kollrobot_controller::PickBoxAction> PickBoxActionServer;

class PickBoxActionClass {

private:
    ros::NodeHandle* _node;
    std::string _actionName = "PickBoxAction";
    PickBoxActionServer* _server;
    KollrobotMoveGroup* _moveGroup;

    kollrobot_controller::PickBoxFeedback _feedback;
    kollrobot_controller::PickBoxResult _result;

    void PublishFeedback(std::string state, float percent);
    void Init();


public:
    PickBoxActionClass(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup);
    void ExecuteActionCallback(const kollrobot_controller::PickBoxGoalConstPtr& goal);

};
