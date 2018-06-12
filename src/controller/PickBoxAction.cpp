//
// Created by thomas on 11.06.18.
//
#include "PickBoxAction.h"

/*
PickBoxAction::PickBoxAction(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup) : _node(node), _moveGroup(moveGroup)
{
    Init();
}
*/
void PickBoxAction::ExecuteAction(kollrobot_controller::PickBoxActionGoalConstPtr& goal, PickBoxActionServer* server)
{
    server->setSucceeded();
}

void PickBoxAction::Init()
{
    //_server = new PickBoxActionServer(*_node, "pickBoxAction", boost::bind(&PickBoxAction::ExecuteAction, this),
    //                                  false);

}
