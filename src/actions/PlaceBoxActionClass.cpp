//
// Created by thomas on 11.06.18.
//
#include "PlaceBoxActionClass.h"

namespace PlaceBoxAction {

    PlaceBoxActionClass::PlaceBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup) :
            BaseActionClass(node, moveGroup) {
        _actionName = "PlaceBoxAction";
        Init();
    }

    void PlaceBoxActionClass::PublishFeedback(std::string state, float percent, bool warn = false) {
        if (!warn) {
            ROS_INFO_STREAM(state);
        } else {
            ROS_ERROR_STREAM(state);
        }

        _feedback.percent_complete = percent;

        _server->publishFeedback(_feedback);
    }

    geometry_msgs::PoseStamped PlaceBoxActionClass::CalculatePrePlacePosition(geometry_msgs::PoseStamped targetPose)
    {
        targetPose = _transformationHandler->TransformPose(targetPose, "base_link");

        _transformationHandler->SendTransform(targetPose, "place_link");
        _transformationHandler->WaitOne();
        targetPose = _transformationHandler->TransformPose(targetPose, "place_link");


        targetPose.pose.position.z += 0.3;
        targetPose.pose.position.x += 0.05;

        return targetPose;
    }

    std::vector<geometry_msgs::PoseStamped>
    PlaceBoxActionClass::CalculatePlacePoseSeries(geometry_msgs::PoseStamped targetPose) {

        auto transform = _transformationHandler->GetTransform(targetPose.header.frame_id, "base_link");

        std::vector<geometry_msgs::PoseStamped> waypoints;
        waypoints.push_back(_transformationHandler->TransformPose(transform, targetPose));

        geometry_msgs::PoseStamped pose1 = targetPose;
        pose1.pose.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));

        geometry_msgs::PoseStamped pose2 = targetPose;
        pose2.pose.position.x = 0.0;
        pose2.pose.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::PoseStamped pose3 = targetPose;
        pose3.pose.position.x = -0.1;
        pose3.pose.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::PoseStamped pose4 = targetPose;
        pose4.pose.position.z = 0.3;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        return waypoints;
    }

    void PlaceBoxActionClass::ExecuteActionCallback(const ActionGoal goal)
    {

        PublishFeedback("Start GoPositionAction!", 0.0);

        if (_moveGroup->IsBusy())
        {
            ROS_INFO_STREAM("Can not run action, movegroup is busy!");
            PublishFeedback("Can't start pick action", 100.0, true);
        }
        PublishFeedback("Moving to pre place position", 0.0);
        geometry_msgs::PoseStamped prePlacePose = CalculatePrePlacePosition(goal->place_pose);
        _moveGroup->PlanToPoseExecute(prePlacePose);
        PublishFeedback("Moved to to pre pick position", 10.0);

        PublishFeedback("Calculating Gripping Trajectory", 20.0);
        auto poseSeries = CalculatePlacePoseSeries(prePlacePose);
        PublishFeedback("Execute Gripping Trajectory", 40.0);
        _moveGroup->ExecutePoseSeries(poseSeries);

        PublishFeedback("Moving back to home position position", 90.0);
        _moveGroup->GoHome();

        PublishFeedback("Finished pick action", 100.0);
        _result.succeed = true;
        _server->setSucceeded(_result);

        return;
    }

}