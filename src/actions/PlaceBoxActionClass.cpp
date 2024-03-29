//
// Created by thomas on 11.06.18.
//
#include "PlaceBoxActionClass.h"

namespace PlaceBoxAction {

    PlaceBoxActionClass::PlaceBoxActionClass(ros::NodeHandle *node, KollrobotMoveGroup *moveGroup) :
            BaseActionClass(node, moveGroup) {
        _actionName = "PlaceBoxAction";
        BaseActionClass::Init();
        Init();
    }

    void PlaceBoxActionClass::PublishFeedback(std::string state, float percent, bool warn = false) {
        if (!warn)
        {
            ROS_INFO_STREAM(state);
        } else {
            ROS_ERROR_STREAM(state);
        }

        _feedback.percent_complete = percent;

        _server->publishFeedback(_feedback);
    }


    void PlaceBoxActionClass::InitParameter()
    {
        paramMaxRange = _parameterHandler->AddParameter("MaxRange", "" , 1.4f);
        paramGripperOffset = _parameterHandler->AddParameter("GripperOffset", "" , 0.068f);
        paramGripperRotOffset = _parameterHandler->AddParameter("GripperRotOffset", "" , 48.0f);
        paramTrajectoryReplanningCount = _parameterHandler->AddParameter("TrajectoryReplanningCount", "", 5);
        paramPlaceXOffset = _parameterHandler->AddParameter("Place_X_Offset", "", 0.065f);
    }

    void PlaceBoxActionClass::Init()
    {
        SetPlaceOrientation();
        InitParameter();
    }

    void PlaceBoxActionClass::SetPlaceOrientation() {

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

        placeOrientation.position.x = 0.00;
        placeOrientation.position.y = 0.00;
        placeOrientation.position.z = 0.00;

        placeOrientation.orientation.x = combined.x();
        placeOrientation.orientation.y = combined.y();
        placeOrientation.orientation.z = combined.z();
        placeOrientation.orientation.w = combined.w();
    }

    bool PlaceBoxActionClass::CheckRange(geometry_msgs::Vector3 position)
    {
        bool ret = false;

        float distance =(float)sqrt(pow(position.x, 2.0) + pow(position.y, 2.0) + pow(position.z, 2.0));
        ROS_INFO_STREAM("Objects distance to robot is: " + std::to_string(distance) + " meter");
        if (distance <= paramMaxRange.GetValue())
            ret = true;

        return ret;
    }

    geometry_msgs::PoseStamped PlaceBoxActionClass::CalculatePrePlacePosition(geometry_msgs::PoseStamped targetPose)
    {
        targetPose = _transformationHandler->TransformPose(targetPose, "base_link");

        _transformationHandler->SendTransform(targetPose, "base_link");
        _transformationHandler->WaitOne();
        targetPose = _transformationHandler->TransformPose(targetPose, "base_link");


        targetPose.pose.position.z += 0.3;
        targetPose.pose.position.x += paramGripperOffset.GetValue();

        return targetPose;
    }

    geometry_msgs::PoseStamped PlaceBoxActionClass::CalculatePrePlacePosition(std::string frameID,
                                                                              geometry_msgs::PoseStamped targetPose)
    {
        geometry_msgs::PoseStamped prePickPose(targetPose);
        prePickPose.header.frame_id = frameID;
        prePickPose.pose.position = targetPose.pose.position;
        prePickPose.pose.position.z -= 0.3;
        prePickPose.pose.position.x += 0.2;
        //force quaternion
        prePickPose.pose.orientation = placeOrientation.orientation;

        return prePickPose;
    }

    std::vector<geometry_msgs::PoseStamped>
    PlaceBoxActionClass::CalculatePlacePoseSeries(geometry_msgs::PoseStamped targetPose,
                                                geometry_msgs::TransformStamped transform) {
        auto gripperOffset = paramGripperOffset.GetValue();
        std::vector<geometry_msgs::PoseStamped> waypoints;

        geometry_msgs::PoseStamped pose1 = targetPose;
        pose1.pose.position.x += 0.16;
        pose1.pose.position.z -= 0.40;

        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));
        geometry_msgs::PoseStamped pose2 = pose1;
        pose2.pose.position.z = -gripperOffset + 0.045;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::PoseStamped pose3 = pose2;
        pose3.pose.position.z -= -0.01;
        pose3.pose.position.x = targetPose.pose.position.x + paramPlaceXOffset.GetValue();
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::PoseStamped pose4 = pose3;
        pose4.pose.position.z -= 0.25;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        return waypoints;
    }

    bool PlaceBoxActionClass::CheckPlaceAvailability(std::string boxFrameID)
    {
        return _transformationHandler->FrameExist(boxFrameID);
    }

    void PlaceBoxActionClass::SetConstraints()
    {
        //Set constraints to keep box up!
        moveit_msgs::Constraints constraints;
        constraints.name = "BoxUP";
        std::vector <std::string> linkNames = _moveGroup->_moveGroup->getLinkNames();

        geometry_msgs::TransformStamped currentPose = _transformationHandler->GetTransform("world", "wrist_2_link");

        moveit_msgs::OrientationConstraint ocm;

        ocm.link_name = "wrist_2_link";
        ocm.header.frame_id = "base_link";
        ocm.header.stamp = ros::Time::now();
        ocm.orientation = currentPose.transform.rotation;

        ocm.absolute_x_axis_tolerance = 2*M_PI;
        ocm.absolute_y_axis_tolerance = 2*M_PI;
        ocm.absolute_z_axis_tolerance = 0.2 * M_PI;
        ocm.weight = 1.0;
        constraints.orientation_constraints.push_back(ocm);

        _moveGroup->SetConstraints(constraints);
    }


    void PlaceBoxActionClass::ExecuteActionCallback(const ActionGoal goal)
    {
        ROS_INFO_STREAM("Startet new " << _actionName);


        SetPlaceOrientation();
        bool success = true;
        if (_moveGroup->IsBusy()) {
            PublishFeedback("MoveGroup is busy! Can't start action!", 0.0);
            success = false;
            _server->setSucceeded(_result);
            return;
        }

        auto newGoal = goal.get();
        if (!CheckPlaceAvailability(newGoal->place_frameID)) {
            std::string msg =
                    "Box Transformation - " + newGoal->place_frameID + " - is not available, can't pick it!";
            PublishFeedback(msg, 0.0);
            success = false;
            _server->setSucceeded(_result);
            return;
        }

        geometry_msgs::TransformStamped boxTransform = _transformationHandler->GetTransform(newGoal->place_frameID,
                                                                                            "base_link");

        if (!CheckRange(boxTransform.transform.translation)) {
            PublishFeedback("Box is not reachable within the workspace! - Aborting PickAction!", 100.0);
            _server->setSucceeded(_result);

            return;
        }


        SetConstraints();
        geometry_msgs::PoseStamped startPose = _moveGroup->GetEndEffectorPose();
        startPose = _transformationHandler->TransformPose(startPose, newGoal->place_frameID);
        geometry_msgs::PoseStamped targetPose = newGoal->place_pose;
        targetPose.pose = placeOrientation;

        PublishFeedback("Calculating Gripping Trajectory", 10.0);
        auto poseSeries = CalculatePlacePoseSeries(targetPose, boxTransform);

        //add home start pose to pose Series
        poseSeries.push_back(startPose);

        //set speeds for the single trajecotry points
        std::vector<float> speeds{ 1.0, 0.5, 0.5, 0.8, 1.0, 1.0};
        auto trajectories = _moveGroup->CalculateTrajectory(poseSeries, speeds);

        bool check = false;
        for (int i = 0; i < paramTrajectoryReplanningCount.GetValue(); i++)
        {
            auto trajectories = _moveGroup->CalculateTrajectory(poseSeries, speeds);
            //auto trajectory = _moveGroup->FuseTrajectories(trajectories);
            check = _moveGroup->CheckTrajecotry(trajectories);

            if(check)
                break;
        }

        if(!check)
        {
            //cancel action
            PublishFeedback("Trajectory is not valid! Can't reach target! Cancel action!", 80.0);
            PublishFeedback("Moving back to home position position", 90.0);
            _moveGroup->GoHome();
            _result.succeed = 0;
            _server->setSucceeded(_result);
            return;
        }

        PublishFeedback("Execute Gripping Trajectory", 20.0);

        //_moveGroup->ExecutePoseSeries(poseSeries);
        _moveGroup->ExecutePoseSeriesAsTrajectory(poseSeries, speeds, "base_link");
        //_moveGroup->ExecuteTrajectory(trajectories, "base_link");

        PublishFeedback("Moving back to home position position", 90.0);
        _moveGroup->GoHome();
        _moveGroup->ClearConstraints();
        PublishFeedback("Finished " + _actionName, 100.0);
        //_result.succeed = success;
        _server->setSucceeded(_result);
        return;
    }

}