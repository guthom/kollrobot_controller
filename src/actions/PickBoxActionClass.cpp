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

    void PickBoxActionClass::Init()
    {

        SetBoxOrientation();
    }

    void PickBoxActionClass::InitParameter()
    {
        paramMaxRange = _parameterHandler->AddParameter("MaxRange", "" , 1.25f);
        paramGripperOffset = _parameterHandler->AddParameter("GripperOffset", "" , 0.068f);
        paramGripperRotOffset = _parameterHandler->AddParameter("GripperRotOffset", "" , 48.0f);
    }

    bool PickBoxActionClass::CheckRange(geometry_msgs::Vector3 position)
    {
        bool ret = false;

        float distance =(float)sqrt(pow(position.x, 2.0) + pow(position.y, 2.0) + pow(position.z, 2.0));
        ROS_INFO_STREAM("Objects distance to robot is: " + std::to_string(distance) + " meter");
        if (distance <= paramMaxRange.GetValue())
            ret = true;

        return ret;
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
        geometry_msgs::PoseStamped currentPose = _moveGroup->GetEndEffectorPose();

        currentPose = _transformationHandler->TransformPose(currentPose, "world", "base_link");

        moveit_msgs::OrientationConstraint ocm;

        ocm.link_name = "ee_link";
        ocm.header.frame_id = "base_link";
        ocm.header.stamp = ros::Time::now();
        ocm.orientation = currentPose.pose.orientation;

        ocm.absolute_x_axis_tolerance = M_PI;
        ocm.absolute_y_axis_tolerance = M_PI;
        ocm.absolute_z_axis_tolerance = 0.5*M_PI; //ignore this axis
        ocm.weight = 1.0;
        constraints.orientation_constraints.push_back(ocm);

        _moveGroup->SetConstraints(constraints);
    }

    void PickBoxActionClass::SetBoxOrientation()
    {
        //TODO: Find a better way to do this
        /*
        boxOrientation.orientation.x = 0.707;
        boxOrientation.orientation.y = 0.0;
        boxOrientation.orientation.z = 0.707;
        boxOrientation.orientation.w = 0.0;
        */

        // hack rotation for gripper
        tf::Quaternion baseQuat, hackedRotation;
        baseQuat.setX(0.707);
        baseQuat.setY(0.0);
        baseQuat.setZ(0.707);
        baseQuat.setW(0.0);

        //deg to rad
        double rad = paramGripperRotOffset.GetValue() * M_PI / 180.0f;

        hackedRotation.setEuler(0.0, rad, 0.0);

        auto combined = baseQuat * hackedRotation;

        boxOrientation.orientation.x = combined.x();
        boxOrientation.orientation.y = combined.y();
        boxOrientation.orientation.z = combined.z();
        boxOrientation.orientation.w = combined.w();
    }

    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(std::string frameID,
                                                                            geometry_msgs::PoseStamped targetPose)
    {
        geometry_msgs::PoseStamped prePickPose(targetPose);
        prePickPose.pose.position = targetPose.pose.position;
        prePickPose.header.frame_id = frameID;
        prePickPose.pose.position.z -= 0.3;
        prePickPose.pose.position.x -= 0.005;
        //force quaternion
        prePickPose.pose.orientation = boxOrientation.orientation;

        return prePickPose;
    }

    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(std::string targetFrame)
    {
        geometry_msgs::PoseStamped prePickPose;

        prePickPose.header.frame_id = targetFrame;
        prePickPose.pose.position.z -= 0.3;
        prePickPose.pose.position.x -= 0.00;
        //force quaternion
        prePickPose.pose.orientation = boxOrientation.orientation;

        return prePickPose;
    }

    moveit_msgs::RobotTrajectory PickBoxActionClass::CalculatePickTrajectory(geometry_msgs::PoseStamped targetPose,
                                                                             geometry_msgs::TransformStamped transform)
    {
        auto gripperOffset = paramGripperOffset.GetValue();

        geometry_msgs::PoseStamped currentPose = _moveGroup->GetEndEffectorPose();
        currentPose = _transformationHandler->TransformPose(currentPose, "world", "base_link");

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(_transformationHandler->TransformPose(transform, targetPose.pose));

        geometry_msgs::Pose pose1 = targetPose.pose;
        pose1.position.z = -gripperOffset;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));

        geometry_msgs::Pose pose2 = pose1;
        pose2.position.x = 0.05;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::Pose pose3 = pose2;
        pose3.position.x = 0.1;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::Pose pose4 = pose3;
        pose4.position.x = 0.1;
        pose4.position.z = -0.3;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        waypoints.push_back(currentPose.pose);

        moveit_msgs::RobotTrajectory trajectory = _moveGroup->ComputeCartesianpath(waypoints, "base_link");

        //ReplanTrajectory(trajectory);

        return trajectory;
    }


    void PickBoxActionClass::ReplanTrajectory(moveit_msgs::RobotTrajectory& trajectory)
    {
        const double accelerationFactor = 0.01;//(double)_parameterHandler->GetFloatParameter("MaxAccelerationScale").GetValue();
        const double velocityFactor =  0.01;//(double)_parameterHandler->GetFloatParameter("MaxVelocityScale").GetValue();

        for(int i = 0; i <= trajectory.joint_trajectory.points.size() -1; i++)
        {
            auto point = trajectory.joint_trajectory.points[i];

            for(int j = 0; j <= point.accelerations.size() -1; j++)
            {
                point.accelerations[j] *= accelerationFactor;
                point.velocities[j] *= velocityFactor;
                point.time_from_start *= 0.5;
            }
        }
    }

    std::vector<geometry_msgs::PoseStamped>
    PickBoxActionClass::CalculatePickPoseSeries(geometry_msgs::PoseStamped targetPose,
                                                geometry_msgs::TransformStamped transform)
    {

        auto gripperOffset = paramGripperOffset.GetValue();
        std::vector<geometry_msgs::PoseStamped> waypoints;
        waypoints.push_back(_transformationHandler->TransformPose(transform, targetPose));

        geometry_msgs::PoseStamped pose1 = targetPose;
        pose1.pose.position.z = -gripperOffset;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));

        geometry_msgs::PoseStamped pose2 = pose1;
        pose2.pose.position.x = 0.01;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::PoseStamped pose3 = pose2;
        pose3.pose.position.x = 0.1;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::PoseStamped pose4 = pose3;
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
        if (!CheckBoxAvailability(newGoal->box_frameID)) {
            std::string msg =
                    "Box Transformation - " + newGoal->box_frameID + " - is not available, can't pick it!";
            PublishFeedback(msg, 0.0);
            success = false;
            _server->setSucceeded(_result);
            return;
        }

        geometry_msgs::TransformStamped boxTransform = _transformationHandler->GetTransform(newGoal->box_frameID,
                                                                                            "base_link");

        if(!CheckRange(boxTransform.transform.translation))
        {
            PublishFeedback("Box is not reachable within the workspace! - Aborting PickAction!", 100.0);

            _server->setSucceeded(_result);

            return;
        }

        SetConstraints();
        PublishFeedback("Calculating pre pick position", 0.0);
        auto prePickPose = CalculatePrePickPosition(newGoal->box_frameID, newGoal->box_pose);
        _moveGroup->PlanToPoseExecute(prePickPose);

        PublishFeedback("Calculating Gripping Trajectory", 10.0);
        auto poseSeries = CalculatePickPoseSeries(prePickPose, boxTransform);
        //auto trajectory = CalculatePickTrajectory(prePickPose, boxTransform);

        PublishFeedback("Execute Gripping Trajectory", 20.0);
        _moveGroup->ExecutePoseSeries(poseSeries);
        //_moveGroup->ExecuteTrajectory(trajectory);

        //PublishFeedback("Moving back to pre pick position", 80.0);
        //_moveGroup->PlanToPoseExecute(prePickPose);

        PublishFeedback("Moving back to home position position", 90.0);
        _moveGroup->GoHome();
        _moveGroup->ClearConstraints();
        PublishFeedback("Finished " + _actionName, 100.0);
        //_result.succeed = success;
        _server->setSucceeded(_result);
        return;
    }
}