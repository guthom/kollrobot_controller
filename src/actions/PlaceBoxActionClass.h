//
// Created by thomas on 11.06.18.
//
#pragma once

#include <ros/ros.h>
#include "../controller/KollrobotMoveGroup.h"
#include "BaseActionClass.h"
#include "../helper/TransformationHandler.h"
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <kollrobot_controller/PlaceBoxAction.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>

namespace PlaceBoxAction
{
    typedef actionlib::SimpleActionServer<kollrobot_controller::PlaceBoxAction> ActionServer;
    typedef kollrobot_controller::PlaceBoxGoalConstPtr ActionGoal;
    typedef kollrobot_controller::PlaceBoxFeedback ActionFeedback;
    typedef kollrobot_controller::PlaceBoxResult ActionResult;

    class PlaceBoxActionClass :
            public BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal> {

    private:

        void PublishFeedback(std::string state, float percent, bool warn);

    public:
        PlaceBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup);

        void ExecuteActionCallback(const ActionGoal goal) override;
    };
}
