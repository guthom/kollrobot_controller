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
#include <kollrobot_controller/GoPositionAction.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>

namespace GoPositionAction
{
    typedef actionlib::SimpleActionServer<kollrobot_controller::GoPositionAction> ActionServer;
    typedef kollrobot_controller::GoPositionGoalConstPtr ActionGoal;
    typedef kollrobot_controller::GoPositionFeedback ActionFeedback;
    typedef kollrobot_controller::GoPositionResult ActionResult;

    class GoPositionActionClass :
            public BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal> {

    private:

        void PublishFeedback(std::string state, float percent, bool warn);

        void Init() override;

    public:
        GoPositionActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup);

        void ExecuteActionCallback(const ActionGoal goal) override;
    };
}
