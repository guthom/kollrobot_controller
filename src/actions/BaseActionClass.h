//
// Created by thomas on 11.06.18.
//
#pragma once

#include <ros/ros.h>
#include "../controller/KollrobotMoveGroup.h"
#include "../helper/TransformationHandler.h"
#include <custom_parameter/parameter.h>
#include <custom_parameter/parameterHandler.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>


template <typename ActionServer, typename ActionFeedback, typename ActionResult, typename ActionGoal>
class BaseActionClass
{

protected:
    ros::NodeHandle* _node;
    ActionServer* _server;
    std::string _actionName;
    KollrobotMoveGroup* _moveGroup;
    TransformationHandler* _transformationHandler;
    customparameter::ParameterHandler *_parameterHandler;
    virtual void InitParameter() {};


    ActionFeedback _feedback;
    ActionResult _result;

    virtual void Init();

public:
    BaseActionClass(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup);
    virtual void ExecuteActionCallback(const ActionGoal goal) = 0;
};


template <typename ActionServer, typename ActionFeedback, typename ActionResult, typename ActionGoal>
BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal>::BaseActionClass(
        ros::NodeHandle* node, KollrobotMoveGroup* moveGroup) : _node(node), _moveGroup(moveGroup)
{
    _transformationHandler = new TransformationHandler(_node);
    _parameterHandler = new customparameter::ParameterHandler(node);
}

template <typename ActionServer, typename ActionFeedback, typename ActionResult, typename ActionGoal>
void BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal>::Init() {

    _server = new ActionServer(*_node, _actionName, boost::bind(&BaseActionClass::ExecuteActionCallback, this,
                                                                _1), false);

    _server->start();
    ROS_INFO_STREAM("Started Action Server for " << _actionName);

    InitParameter();

}

