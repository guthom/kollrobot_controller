//
// Created by thomas on 11.06.18.
//
#pragma once

#include <ros/ros.h>
#include "KollrobotMoveGroup.h"
#include "../helper/TransformationHandler.h"
#include <kollrobot_controller/PickBoxAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<kollrobot_controller::PickBoxAction> PickBoxActionServer;

class PickBoxActionClass {

private:
    ros::NodeHandle* _node;
    std::string _actionName = "PickBoxAction";
    PickBoxActionServer* _server;
    KollrobotMoveGroup* _moveGroup;
    TransformationHandler _transformationHandler;

    kollrobot_controller::PickBoxFeedback _feedback;
    kollrobot_controller::PickBoxResult _result;

    bool CheckBoxAvailability(std::string boxFrameID);

    void PublishFeedback(std::string state, float percent, bool warn);
    void Init();

    //methods to calculate positions/trajecotries ect
    geometry_msgs::PoseStamped CalculatePrePickPosition(geometry_msgs::PoseStamped targetPose);
    moveit_msgs::RobotTrajectory CalculatePickTrajectory(geometry_msgs::PoseStamped targetPose);

public:
    PickBoxActionClass(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup);
    void ExecuteActionCallback(const kollrobot_controller::PickBoxGoalConstPtr& goal);

};
