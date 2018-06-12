//
// Created by thomas on 11.06.18.
//

#ifndef PICKBOXACTION_H
#define PICKBOXACTION_H

#include <ros/ros.h>
//#include "KollrobotMoveGroup.h"
#include <kollrobot_controller/PickBoxAction.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<kollrobot_controller::PickBoxAction> PickBoxActionServer;

class PickBoxAction {

private:
    ros::NodeHandle* _node;
    PickBoxActionServer* _server;
    //KollrobotMoveGroup* _moveGroup;

    void Init();
    void ExecuteAction(kollrobot_controller::PickBoxActionGoalConstPtr& goal, PickBoxActionServer* server);

public:
    //PickBoxAction(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup);

};


#endif //PROJECT_PICKBOXACTION_H
