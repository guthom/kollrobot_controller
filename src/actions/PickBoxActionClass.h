//
// Created by thomas on 11.06.18.
//
#pragma once

#include <ros/ros.h>
#include "BaseActionClass.h"
#include "../controller/KollrobotMoveGroup.h"
#include "../helper/TransformationHandler.h"
#include <kollrobot_controller/PickBoxAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <vector>

#include <actionlib/server/simple_action_server.h>

namespace PickBoxAction
{
    typedef actionlib::SimpleActionServer<kollrobot_controller::PickBoxAction> ActionServer;
    typedef kollrobot_controller::PickBoxGoalConstPtr ActionGoal;
    typedef kollrobot_controller::PickBoxFeedback ActionFeedback;
    typedef kollrobot_controller::PickBoxResult ActionResult;

    class PickBoxActionClass :
            public BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal>
    {

    private:
        bool CheckBoxAvailability(std::string boxFrameID);
        void PublishFeedback(std::string state, float percent, bool warn);

        geometry_msgs::Pose pickOrientation;
        void Init();
        void InitParameter();

        bool CheckRange(geometry_msgs::Vector3 position);

        customparameter::Parameter<float> paramMaxRange;

        //methods to calculate positions/trajecotries ect
        void SetPickOrientation();
        void SetConstraints();
        geometry_msgs::PoseStamped CalculatePrePickPosition(std::string frameID,
                                                            geometry_msgs::PoseStamped targetPose);
        geometry_msgs::PoseStamped CalculatePrePickPosition(std::string targetFrame);
        moveit_msgs::RobotTrajectory CalculatePickTrajectory(geometry_msgs::PoseStamped targetPose);
        std::vector<geometry_msgs::PoseStamped> CalculatePickPoseSeries(geometry_msgs::PoseStamped targetPose,
                                                                        geometry_msgs::TransformStamped transform);

    public:
        PickBoxActionClass(ros::NodeHandle* node, KollrobotMoveGroup* moveGroup);
        void ExecuteActionCallback(const ActionGoal goal) override;

    };
}