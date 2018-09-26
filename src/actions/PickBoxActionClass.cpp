//
// Created by thomas on 11.06.18.
//
#include "PickBoxActionClass.h"

namespace PickBoxAction {

    PickBoxActionClass::PickBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup) :
            BaseActionClass(node, moveGroup) {
        _actionName = "PickBoxAction";
        BaseActionClass::Init();
        Init();
    }

    void PickBoxActionClass::Init() {
    }

    void PickBoxActionClass::PublishFeedback(std::string state, float percent, bool warn = false) {
        if (!warn) {
            ROS_INFO_STREAM(state);
        } else {
            ROS_ERROR_STREAM(state);
        }

        _feedback.current_state = state;
        _feedback.percent_complete = percent;

        _server->publishFeedback(_feedback);
    }

    void PickBoxActionClass::SetConstraints()
    {
        //Set constraints to keep box up!
        moveit_msgs::Constraints constraints;
        constraints.name = "BoxUP";
        std::vector <std::string> linkNames = _moveGroup->_moveGroup->getLinkNames();
        geometry_msgs::PoseStamped currentPose = _moveGroup->_moveGroup->getCurrentPose(
                _moveGroup->_moveGroup->getEndEffectorLink());
        currentPose = _transformationHandler->TransformPose(currentPose, "world", "base_link");

        moveit_msgs::OrientationConstraint ocm;

        ocm.link_name = "wrist_3_link";
        ocm.header.frame_id = "base_link";
        ocm.header.stamp = ros::Time::now();
        ocm.orientation = currentPose.pose.orientation;


        ocm.orientation = currentPose.pose.orientation;

        ocm.absolute_x_axis_tolerance = M_PI;
        ocm.absolute_y_axis_tolerance = M_PI;
        ocm.absolute_z_axis_tolerance = M_PI; //ignore this axis

        ocm.weight = 1.0;
        constraints.orientation_constraints.push_back(ocm);

        //_moveGroup->SetConstraints(constraints);

    }

    void PickBoxActionClass::SetPickOrientation(std::string targetFrame)
    {
        //TODO: Find a better way to do this
        geometry_msgs::PoseStamped tempPose = _moveGroup->GetEndEffectorPose();
        tempPose.header.frame_id = "base_link";
        tempPose = _transformationHandler->TransformPose(tempPose, targetFrame);
        pickOrientation.orientation = tempPose.pose.orientation;
    }

    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(std::string frameID,
                                                                            geometry_msgs::PoseStamped targetPose)
    {
        SetPickOrientation(frameID);

        geometry_msgs::PoseStamped prePickPose(targetPose);
        prePickPose.pose.position.z -= 0.3;
        prePickPose.pose.position.x -= 0.05;
        //force quaternion
        prePickPose.pose.orientation = pickOrientation.orientation;

        return prePickPose;
    }

    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(std::string targetFrame)
    {
        SetPickOrientation(targetFrame);

        geometry_msgs::PoseStamped prePickPose;
        prePickPose.header.frame_id = targetFrame;
        prePickPose.pose.position.z -= 0.3;
        prePickPose.pose.position.x -= 0.05;
        //force quaternion
        prePickPose.pose.orientation = pickOrientation.orientation;

        return prePickPose;
    }

    moveit_msgs::RobotTrajectory PickBoxActionClass::CalculatePickTrajectory(geometry_msgs::PoseStamped prePickPose) {
        auto transform = _transformationHandler->GetTransform(prePickPose.header.frame_id, "base_link");

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(_transformationHandler->TransformPose(transform, prePickPose.pose));

        geometry_msgs::Pose pose1 = prePickPose.pose;
        pose1.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));

        geometry_msgs::Pose pose2 = prePickPose.pose;
        pose2.position.x = 0.0;
        pose2.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::Pose pose3 = prePickPose.pose;
        pose3.position.x = 0.1;
        pose3.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::Pose pose4 = prePickPose.pose;
        pose4.position.x = 0.1;
        pose4.position.z = -0.3;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        moveit_msgs::RobotTrajectory trajecotry = _moveGroup->ComputeCartesianpath(waypoints);

        return trajecotry;
    }


    std::vector<geometry_msgs::PoseStamped>
    PickBoxActionClass::CalculatePickPoseSeries(geometry_msgs::PoseStamped targetPose,
                                                geometry_msgs::TransformStamped transform)
    {

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
        pose3.pose.position.x = 0.1;
        pose3.pose.position.z = 0.0;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::PoseStamped pose4 = targetPose;
        pose4.pose.position.x = 0.1;
        pose4.pose.position.z = -0.3;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        return waypoints;
    }


    bool PickBoxActionClass::CheckBoxAvailability(std::string boxFrameID) {
        return _transformationHandler->FrameExist(boxFrameID);
    }

    void PickBoxActionClass::ExecuteActionCallback(const ActionGoal goal) {
        ROS_INFO_STREAM("Startet new " << _actionName);


        bool success = true;
        if (_moveGroup->IsBusy()) {
            PublishFeedback("MoveGroup is busy! Can't start action!", 0.0);
            success = false;
            _server->setSucceeded(_result);
            return;
        }

        auto newGoal = goal.get();
        SetConstraints();
        if (!CheckBoxAvailability(newGoal->box_pose.header.frame_id)) {
            std::string msg =
                    "Box Transformation - " + newGoal->box_pose.header.frame_id + " - is not available, can't pick it!";
            PublishFeedback(msg, 0.0);
            success = false;
            _server->setSucceeded(_result);
            return;
        }

        geometry_msgs::TransformStamped boxTransform = _transformationHandler->GetTransform(newGoal->box_frameID,
                                                                                            "base_link");

        PublishFeedback("Moving to pre pick position", 0.0);
        auto prePickPose = CalculatePrePickPosition(newGoal->box_frameID, newGoal->box_pose);
        _moveGroup->PlanToPoseExecute(prePickPose);
        PublishFeedback("Moved to to pre pick position", 10.0);

        PublishFeedback("Calculating Gripping Trajectory", 20.0);
        auto poseSeries = CalculatePickPoseSeries(prePickPose, boxTransform);
        //auto trajectory = CalculatePickTrajectory(prePickPose);


        PublishFeedback("Execute Gripping Trajectory", 40.0);
        _moveGroup->ExecutePoseSeries(poseSeries);
        //_moveGroup->ExecuteTrajectory(trajectory);

        //PublishFeedback("Moving back to pre pick position", 80.0);
        //_moveGroup->PlanToPoseExecute(prePickPose);

        PublishFeedback("Moving back to home position position", 90.0);
        _moveGroup->GoHome();
        _moveGroup->ClearConstraints();
        PublishFeedback("Finished pick action", 100.0);
        //_result.succeed = success;
        _server->setSucceeded(_result);
        return;
    }
}