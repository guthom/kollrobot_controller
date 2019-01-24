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

        SetBoxOrientation();
    }

    void PickBoxActionClass::InitParameter() {
        paramMaxRange = _parameterHandler->AddParameter("MaxRange", "", 1.4f);
        paramGripperOffset = _parameterHandler->AddParameter("GripperOffset", "", 0.068f);
        paramGripperRotOffset = _parameterHandler->AddParameter("GripperRotOffset", "", 48.0f);
        paramGrippingTilt = _parameterHandler->AddParameter("GrippingTilt", "", 5.0f);
        paramTrajectoryReplanningCount = _parameterHandler->AddParameter("TrajectoryReplanningCount", "", 5);
    }

    bool PickBoxActionClass::CheckRange(geometry_msgs::Vector3 position) {
        bool ret = false;

        float distance = (float) sqrt(pow(position.x, 2.0) + pow(position.y, 2.0) + pow(position.z, 2.0));
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

        geometry_msgs::TransformStamped currentPose = _transformationHandler->GetTransform("wrist_2_link", "world");

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


    void PickBoxActionClass::SetBoxOrientation() {
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


    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(geometry_msgs::PoseStamped targetPose,
                                                                            geometry_msgs::TransformStamped transform) {
        geometry_msgs::PoseStamped prePickPose(targetPose);
        prePickPose.pose.position = targetPose.pose.position;
        prePickPose.header.frame_id = transform.child_frame_id;
        prePickPose.pose.position.z -= 0.15;
        prePickPose.pose.position.x -= 0.005;
        //force quaternion
        prePickPose.pose.orientation = boxOrientation.orientation;

        return prePickPose;
    }

    geometry_msgs::PoseStamped PickBoxActionClass::CalculatePrePickPosition(std::string frameID,
                                                                            geometry_msgs::PoseStamped targetPose) {
        geometry_msgs::PoseStamped prePickPose(targetPose);
        prePickPose.pose.position = targetPose.pose.position;
        prePickPose.header.frame_id = frameID;
        prePickPose.pose.position.z -= 0.15;
        prePickPose.pose.position.x -= 0.005;
        //force quaternion
        prePickPose.pose.orientation = boxOrientation.orientation;

        return prePickPose;
    }


    geometry_msgs::Pose
    PickBoxActionClass::GetTiltOrientation(geometry_msgs::Pose pose)
    {

        double gripperTilt = paramGrippingTilt.GetValue() * M_PI / 180.0f;
        double gripperRot = paramGripperRotOffset.GetValue() * M_PI / 180.0f;

        tf::Quaternion baseQuat, tilt, rot;
        baseQuat.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tilt.setEuler(gripperTilt, 0.0, 0.0);
        rot.setEuler(0.0, -gripperRot, 0.0);

        auto combined =  baseQuat * rot;
        combined *= tilt;
        combined *= rot.inverse();

        combined.normalize();
        pose.orientation.x = combined.x();
        pose.orientation.y = combined.y();
        pose.orientation.z = combined.z();
        pose.orientation.w = combined.w();

        return pose;
    }

    std::vector<geometry_msgs::PoseStamped>
    PickBoxActionClass::CalculatePickPoseSeries(geometry_msgs::PoseStamped targetPose,
                                                geometry_msgs::TransformStamped transform) {
        auto gripperOffset = paramGripperOffset.GetValue();

        targetPose.pose = GetTiltOrientation(targetPose.pose);

        std::vector<geometry_msgs::PoseStamped> waypoints;

        geometry_msgs::PoseStamped pose1 = targetPose;
        pose1.pose.position.z -= 0.15;
        pose1.pose.position.x = -0.013;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose1));

        geometry_msgs::PoseStamped pose2 = pose1;
        pose2.pose.position.z = -gripperOffset + 0.01;
        pose2.pose.position.x -= 0.005;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose2));

        geometry_msgs::PoseStamped pose3 = pose2;
        pose3.pose.position.z -= -0.01;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose3));

        geometry_msgs::PoseStamped pose4 = pose3;
        pose4.pose.orientation = boxOrientation.orientation;
        pose4.pose.position.x += 0.07;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose4));

        geometry_msgs::PoseStamped pose5 = pose4;
        pose5.pose.position.z -= 0.45;
        waypoints.push_back(_transformationHandler->TransformPose(transform, pose5));


        return waypoints;
    }


    bool PickBoxActionClass::CheckBoxAvailability(std::string boxFrameID) {
        return _transformationHandler->FrameExist(boxFrameID);
    }

    bool PickBoxActionClass::ExecutePoseSeries(std::vector<geometry_msgs::PoseStamped> waypoints) {
        _moveGroup->PublishWaypoints(waypoints);

        for (int i = 0; i < waypoints.size(); i++) {
            _moveGroup->PlanToPoseExecute(waypoints[i]);
        }
    }


    void PickBoxActionClass::ExecuteActionCallback(const ActionGoal goal)
    {
        ROS_INFO_STREAM("Startet new " << _actionName);


        //_moveGroup->GoHome();
        SetBoxOrientation();
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

        std::string savePose = _moveGroup->GetSaveStartPosition(boxTransform);
        //_moveGroup->GoPosition(savePose);

        if (!CheckRange(boxTransform.transform.translation)) {
            PublishFeedback("Box is not reachable within the workspace! - Aborting PickAction!", 100.0);
            _server->setSucceeded(_result);

            return;
        }


        SetConstraints();

        geometry_msgs::PoseStamped startPose = _moveGroup->GetEndEffectorPose();
        startPose = _transformationHandler->TransformPose(startPose, newGoal->box_frameID);
        geometry_msgs::PoseStamped targetPose = newGoal->box_pose;
        targetPose.pose = boxOrientation;

        PublishFeedback("Calculating Gripping Trajectory", 10.0);
        auto poseSeries = CalculatePickPoseSeries(targetPose, boxTransform);
        //add home start pose to pose Series
        poseSeries.push_back(startPose);

        //set speeds for the single trajecotry points
        std::vector<float> speeds{ 1.0, 0.5, 0.5, 0.5, 0.8, 1.0, 1.0};

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

        PublishFeedback("Moving back to home position position", 90.0);

        _moveGroup->GoPosition(savePose);
        _moveGroup->GoHome();
        _moveGroup->ClearConstraints();
        PublishFeedback("Finished " + _actionName, 100.0);
        //_result.succeed = success;
        _server->setSucceeded(_result);
    }
}
