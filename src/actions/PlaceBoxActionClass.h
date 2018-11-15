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
            public BaseActionClass<ActionServer, ActionFeedback, ActionResult, ActionGoal>
    {

    private:

        bool CheckPlaceAvailability(std::string boxFrameID);
        void PublishFeedback(std::string state, float percent, bool warn);

        geometry_msgs::Pose placeOrientation;
        geometry_msgs::Pose placeOffset;
        void SetPlaceOrientation();
        geometry_msgs::PoseStamped CalculatePrePlacePosition(geometry_msgs::PoseStamped targetPose);

        void InitParameter();
        void Init();

        //methods to calculate positions/trajecotries ect
        void SetConstraints();

        geometry_msgs::PoseStamped CalculatePrePlacePosition(std::string frameID,
                                                            geometry_msgs::PoseStamped targetPose);
        moveit_msgs::RobotTrajectory CalculatePlaceTrajectory(std::vector<geometry_msgs::PoseStamped> poseSeries);
        std::vector<geometry_msgs::PoseStamped> CalculatePlacePoseSeries(geometry_msgs::PoseStamped targetPose,
                                                                        geometry_msgs::TransformStamped transform);

        bool CheckRange(geometry_msgs::Vector3 position);

        customparameter::Parameter<float> paramMaxRange;
        customparameter::Parameter<float> paramGripperOffset;
        customparameter::Parameter<float> paramGripperRotOffset;

    public:
        PlaceBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup);

        void ExecuteActionCallback(const ActionGoal goal) override;
    };
}
